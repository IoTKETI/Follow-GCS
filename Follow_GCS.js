/**
 * Created by Wonseok Jung in KETI on 2023-03-25.
 */

const mqtt = require("mqtt");
const {nanoid} = require("nanoid");
const fs = require("fs");
const {SerialPort} = require("serialport");
const {ReadlineParser} = require("@serialport/parser-readline");
const GPS = require("gps");

const mavlink = require('./mavlibrary/mavlink');

let gpsPort = null;
let gpsPortNum = 'COM5';
let gpsBaudrate = '9600';
gpsPortOpening();

const gps = new GPS();

const parser = gpsPort.pipe(new ReadlineParser());

let drone_info = {};

let fc = {
    "heartbeat": {},
    "global_position_int": {
        "lat": 0,
        "lon": 0,
        "relative_alt": 20
    }
};

let mqtt_client = null;
let drone_topic = '';
let cmd_topic = '';

let gcs_lat = 0;
let gcs_lon = 0;

let my_sortie_name = 'unknown';
let flag_base_mode = 0;

function gpsPortOpening() {
    if (gpsPort == null) {
        gpsPort = new SerialPort({
            path: gpsPortNum,
            baudRate: parseInt(gpsBaudrate, 10),
        });

        gpsPort.on('open', gpsPortOpen);
        gpsPort.on('close', gpsPortClose);
        gpsPort.on('error', gpsPortError);
        // gpsPort.on('data', gpsPortData);
    } else {
        if (gpsPort.isOpen) {
            gpsPort.close();
            gpsPort = null;
            setTimeout(gpsPortOpening, 2000);
        } else {
            gpsPort.open();
        }
    }
}

function gpsPortOpen() {
    console.log('gpsPort ' + gpsPort.path + ' Data rate: ' + gpsPort.baudRate + ' open.');
}

function gpsPortClose() {
    console.log('gpsPort closed.');

    setTimeout(gpsPortOpening, 2000);
}

function gpsPortError(error) {
    console.log('[gpsPort error]: ' + error.message);

    setTimeout(gpsPortOpening, 2000);
}

gps.on("data", data => {
    if (data.type === 'GGA') {
        console.log('GGA', data);
        if (data.quality != null) {
            gcs_lat = data.lat;
            gcs_lon = data.lon;
            // gcs_alt = data.alt;
        }
    } else if (data.type === 'VTG') {
        console.log('VTG', data);
    }
});

parser.on("data", data => {
    // console.log('parser', data)
    gps.update(data);
});

init();

function init() {
    try {
        drone_info = JSON.parse(fs.readFileSync('../drone_info.json', 'utf8'));
    } catch (e) {
        console.log('can not find [ ../drone_info.json ] file');
        drone_info.id = "Dione";
        drone_info.approval_gcs = "MUV";
        drone_info.host = "121.137.228.240";
        drone_info.drone = "Drone1";
        drone_info.gcs = "KETI_GCS";
        drone_info.type = "ardupilot";
        drone_info.system_id = 1;
        drone_info.gcs_ip = "192.168.1.150";
    }

    drone_topic = '/Mobius/' + drone_info.gcs + '/Drone_Data/' + drone_info.drone;
    cmd_topic = '/Mobius/' + drone_info.gcs + '/GCS_Data/' + drone_info.drone;

    mqtt_connect(drone_info.host, drone_topic);

    setInterval(send_change_home_position_command, 2000, drone_info.drone, cmd_topic, drone_info.system_id);
}

function mqtt_connect(serverip, d_topic) {
    if (mqtt_client === null) {
        let connectOptions = {
            host: serverip,
            port: 1883,
            protocol: "mqtt",
            keepalive: 10,
            clientId: 'TELE_LTE_' + nanoid(15),
            protocolId: "MQTT",
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 2000,
            connectTimeout: 30000,
            queueQoSZero: false,
            rejectUnauthorized: false
        }

        mqtt_client = mqtt.connect(connectOptions);

        mqtt_client.on('connect', function () {
            console.log('mqtt is connected to (' + serverip + ')');

            if (d_topic !== '') {  // GCS topic
                mqtt_client.subscribe(d_topic + '/#', function () {
                    console.log('[mqtt_connect] drone_topic is subscribed: ' + d_topic + '/#');
                });
            }
        });

        mqtt_client.on('message', function (topic, message) {
            if (topic.includes(d_topic)) {
                let _msg = message.toString('hex');
                parseMavFromDrone(_msg);
            }
        })

        mqtt_client.on('error', function (err) {
            console.log('[mqtt] (error) ' + err.message);
        })
    }
}

let mavVersion = 'v1';

function parseMavFromDrone(mavPacket) {
    try {
        let ver = mavPacket.substring(0, 2);
        let sys_id = '';
        let msg_id;
        let base_offset;

        if (ver === 'fd') {
            sys_id = parseInt(mavPacket.substring(10, 12).toLowerCase(), 16);
            msg_id = parseInt(mavPacket.substring(18, 20) + mavPacket.substring(16, 18) + mavPacket.substring(14, 16), 16);
            mavVersion = 'v2';
            base_offset = 20;
        } else {
            sys_id = parseInt(mavPacket.substring(6, 8).toLowerCase(), 16);
            msg_id = parseInt(mavPacket.substring(10, 12).toLowerCase(), 16);
            mavVersion = 'v1';
            base_offset = 12;
        }

        if (msg_id === mavlink.MAVLINK_MSG_ID_HEARTBEAT) { // #00 : HEARTBEAT
            let custom_mode = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let type = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            let autopilot = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            let base_mode = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            let system_status = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();
            base_offset += 2;
            let mavlink_version = mavPacket.substring(base_offset, base_offset + 2).toLowerCase();

            fc.heartbeat = {};
            fc.heartbeat.type = Buffer.from(type, 'hex').readUInt8(0);
            if (fc.heartbeat.type !== mavlink.MAV_TYPE_ADSB && sys_id === parseInt(drone_info.system_id)) {
                fc.heartbeat.autopilot = Buffer.from(autopilot, 'hex').readUInt8(0);
                fc.heartbeat.base_mode = Buffer.from(base_mode, 'hex').readUInt8(0);
                fc.heartbeat.custom_mode = Buffer.from(custom_mode, 'hex').readUInt32LE(0);
                fc.heartbeat.system_status = Buffer.from(system_status, 'hex').readUInt8(0);
                fc.heartbeat.mavlink_version = Buffer.from(mavlink_version, 'hex').readUInt8(0);

                let armStatus = (fc.heartbeat.base_mode & 0x80) === 0x80;

                if (my_sortie_name === 'unknown') {
                    if (armStatus) {
                        flag_base_mode++;
                        if (flag_base_mode === 3) {
                            my_sortie_name = 'arm';
                        }
                    } else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                    }
                } else if (my_sortie_name === 'disarm') {
                    if (armStatus) {
                        flag_base_mode++;
                        if (flag_base_mode === 3) {
                            my_sortie_name = 'arm';
                            gcs_lat = (fc.global_position_int.lat * 10000000).toFixed(0);
                            gcs_lon = (fc.global_position_int.lon * 10000000).toFixed(0);
                        }
                    } else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                        gcs_lat = (fc.global_position_int.lat * 10000000).toFixed(0);
                        gcs_lon = (fc.global_position_int.lon * 10000000).toFixed(0);
                    }
                } else if (my_sortie_name === 'arm') {
                    if (armStatus) {
                        my_sortie_name = 'arm';
                    } else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                        gcs_lat = (fc.global_position_int.lat * 10000000).toFixed(0);
                        gcs_lon = (fc.global_position_int.lon * 10000000).toFixed(0);
                    }
                }
            }
        } else if (msg_id === mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            // var time_boot_ms = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var lat = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var lon = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            var relative_alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();

            if (sys_id === parseInt(drone_info.system_id)) {
                fc.global_position_int = {};
                fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0) / 10000000;
                fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0) / 10000000;
                fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0) / 1000;
                fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0) / 1000;
            }
        }
    } catch
        (e) {
        console.log('[parseMavFromDrone Error]', e);
    }
}

function send_change_home_position_command(target_name, pub_topic, target_sys_id) {
    // let h_pos = get_point_dist((gcs_lat / 10000000), (gcs_lon / 10000000), 0.005, gcs_hdg);
    // gcs_lat = parseInt((h_pos.lat * 10000000).toFixed(0));
    // gcs_lon = parseInt((h_pos.lon * 10000000).toFixed(0));

    // gcs_lat = 374026844;
    // gcs_lon = 1271600130;
    // gcs_lat = 374036621;
    // gcs_lon = 1271617624;
    // gcs_lat = 374031467;
    // gcs_lon = 1271608312;

    var btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.target_component = 1;
    btn_params.frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    btn_params.command = mavlink.MAV_CMD_DO_SET_HOME;
    btn_params.current = 0;
    btn_params.autocontinue = 0;
    btn_params.param1 = 0;
    btn_params.param2 = 0;
    btn_params.param3 = 0;
    btn_params.param4 = 0;
    btn_params.x = gcs_lat;
    btn_params.y = gcs_lon;
    btn_params.z = 0; // altitude of GCS

    try {
        var msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_COMMAND_INT, btn_params);
        if (msg == null) {
            console.log("mavlink message is null");
        } else {
            console.log(gcs_lat, gcs_lon, btn_params.x, btn_params.y);
            mqtt_client.publish(pub_topic, msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id, mavVersion);
    try {
        var mavMsg = null;
        var genMsg = null;
        //var targetSysId = sysId;
        // eslint-disable-next-line no-unused-vars
        //var targetCompId = (params.targetCompId === undefined) ? 0 : params.targetCompId;

        switch (type) {
            // MESSAGE ////////////////////////////////////
            case mavlink.MAVLINK_MSG_ID_COMMAND_LONG:
                mavMsg = new mavlink.messages.command_long(
                    params.target_system,
                    params.target_component,
                    params.command,
                    params.confirmation,
                    params.param1,
                    params.param2,
                    params.param3,
                    params.param4,
                    params.param5,
                    params.param6,
                    params.param7);
                break;
            case mavlink.MAVLINK_MSG_ID_COMMAND_INT:
                mavMsg = new mavlink.messages.command_int(
                    params.target_system,
                    params.target_component,
                    params.frame,
                    params.command,
                    params.current,
                    params.autocontinue,
                    params.param1,
                    params.param2,
                    params.param3,
                    params.param4,
                    params.x,
                    params.y,
                    params.z);
                break;
            case mavlink.MAVLINK_MSG_ID_SET_MODE:
                mavMsg = new mavlink.messages.set_mode(
                    params.target_system,
                    params.base_mode,
                    params.custom_mode);
                break;
            case mavlink.MAVLINK_MSG_ID_MISSION_ITEM:
                mavMsg = new mavlink.messages.mission_item(
                    params.target_system,
                    params.target_component,
                    params.seq,
                    params.frame,
                    params.command,
                    params.current,
                    params.autocontinue,
                    params.param1,
                    params.param2,
                    params.param3,
                    params.param4,
                    params.param5,
                    params.param6,
                    params.param7,
                    params.mission_type);
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
        // console.log('>>>>> MAVLINK OUTGOING MSG: ' + genMsg.toString('hex'));
    }

    return genMsg;
}

function calcDistance(x1, y1, a1, x2, y2, a2) {
    /*
        x1: Latitude of the first point
        y1: Longitude of the first point
        a1: Altitude of the first point
        x2: Latitude of the second point
        y2: Longitude of the second point
        a2: Altitude of the second point
    */
    const R = 6371e3; // R is earth’s radius(metres) (mean radius = 6,371km)
    const φ1 = x1 * Math.PI / 180; // φ(latitude), λ(longitude) in radians
    const φ2 = x2 * Math.PI / 180;
    const Δφ = (x2 - x1) * Math.PI / 180;
    const Δλ = (y2 - y1) * Math.PI / 180;

    const a = Math.sin(Δφ / 2) * Math.sin(Δφ / 2) +
        Math.cos(φ1) * Math.cos(φ2) *
        Math.sin(Δλ / 2) * Math.sin(Δλ / 2);
    const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

    let d = R * c + Math.sqrt(Math.pow(a1 - a2, 2)); // in metres

    return d
}

const get_point_dist = (latitude, longitude, distanceInKm, bearingInDegrees) => {
    const R = 6378.1;
    const dr = Math.PI / 180;
    const bearing = bearingInDegrees * dr;
    let lat = latitude * dr;
    let lon = longitude * dr;

    lat = Math.asin(Math.sin(lat) * Math.cos(distanceInKm / R) + Math.cos(lat) * Math.sin(distanceInKm / R) * Math.cos(bearing));
    lon += Math.atan2(
        Math.sin(bearing) * Math.sin(distanceInKm / R) * Math.cos(lat),
        Math.cos(distanceInKm / R) - Math.sin(lat) * Math.sin(lat)
    );
    lat /= dr;
    lon /= dr;
    return {lat, lon};
}
