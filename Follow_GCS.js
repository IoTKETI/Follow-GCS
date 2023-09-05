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

let mobius_mqtt_client = null;
let drone_topic = '';
let cmd_topic = '';

let gcs_lat = 0;
let gcs_lon = 0;

let my_sortie_name = 'unknown';
let flag_base_mode = 0;

function gpsPortOpening() {
    if (!gpsPort) {
        gpsPort = new SerialPort({
            path: gpsPortNum,
            baudRate: parseInt(gpsBaudrate, 10),
        });

        gpsPort.on('open', gpsPortOpen);
        gpsPort.on('close', gpsPortClose);
        gpsPort.on('error', gpsPortError);
        // gpsPort.on('data', gpsPortData);
    }
    else {
        if (gpsPort.isOpen) {
            gpsPort.close();
            gpsPort = null;
            setTimeout(gpsPortOpening, 2000);
        }
        else {
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
        if (data.quality) {
            gcs_lat = data.lat;
            gcs_lon = data.lon;
            // gcs_alt = data.alt;
        }
    }
    else if (data.type === 'VTG') {
        // console.log('VTG', data);
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
    }
    catch (e) {
        console.log('can not find [ ../drone_info.json ] file');
        drone_info.id = "Dione";
        drone_info.approval_gcs = "MUV";
        drone_info.host = "gcs.iotocean.org";
        drone_info.drone = "Drone1";
        drone_info.gcs = "KETI_GCS";
        drone_info.type = "ardupilot";
        drone_info.system_id = 1;
        drone_info.gcs_ip = "192.168.1.150";
    }

    drone_topic = '/Mobius/' + drone_info.gcs + '/Drone_Data/' + drone_info.drone;
    cmd_topic = '/Mobius/' + drone_info.gcs + '/GCS_Data/' + drone_info.drone;

    mobius_mqtt_connect(drone_info.host, drone_topic);

    setInterval(send_change_home_position_command, 2000, drone_info.drone, cmd_topic, drone_info.system_id);
}

function mobius_mqtt_connect(serverip, d_topic) {
    if (!mobius_mqtt_client) {
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

        mobius_mqtt_client = mqtt.connect(connectOptions);

        mobius_mqtt_client.on('connect', () => {
            console.log('mobius_mqtt_client is connected to (' + serverip + ')');

            if (d_topic !== '') {  // GCS topic
                mobius_mqtt_client.subscribe(d_topic + '/#', () => {
                    console.log('[mobius_mqtt_client] drone_topic is subscribed: ' + d_topic + '/#');
                });
            }
        });

        mobius_mqtt_client.on('message', (topic, message) => {
            if (topic.includes(d_topic)) {
                let _msg = message.toString('hex');
                parseMavFromDrone(_msg);
            }
        });

        mobius_mqtt_client.on('error', (err) => {
            console.log('[mobius_mqtt_client] (error) ' + err.message);
        });
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
        }
        else {
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
                    }
                    else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                    }
                }
                else if (my_sortie_name === 'disarm') {
                    if (armStatus) {
                        flag_base_mode++;
                        if (flag_base_mode === 3) {
                            my_sortie_name = 'arm';
                        }
                    }
                    else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                    }
                }
                else if (my_sortie_name === 'arm') {
                    if (armStatus) {
                        my_sortie_name = 'arm';
                    }
                    else {
                        flag_base_mode = 0;
                        my_sortie_name = 'disarm';
                    }
                }
            }
        }
        else if (msg_id === mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
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
        else if (msg_id === mavlink.MAVLINK_MSG_ID_HOME_POSITION) {
            let lat = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let lon = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let relative_alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            fc.home_position = {};
            fc.home_position.lat = Buffer.from(lat, 'hex').readInt32LE(0) / 10000000;
            fc.home_position.lon = Buffer.from(lon, 'hex').readInt32LE(0) / 10000000;
            fc.home_position.alt = Buffer.from(alt, 'hex').readInt32LE(0) / 1000;
            fc.home_position.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0) / 1000;

            console.log('target Drone Home_Position -', JSON.stringify(fc.home_position));
        }
    }
    catch
        (e) {
        console.log('[parseMavFromDrone Error]', e);
    }
}

function send_change_home_position_command(target_name, pub_topic, target_sys_id) {
    if ((33 < gcs_lat && gcs_lat < 43) && ((124 < gcs_lon && gcs_lon < 132))) {
        let btn_params = {};
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
        btn_params.x = gcs_lat * 10000000;
        btn_params.y = gcs_lon * 10000000;
        btn_params.z = 0; // altitude of GCS

        try {
            let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_COMMAND_INT, btn_params);
            if (!msg) {
                console.log("[send_change_home_position_command] mavlink message is null");
            }
            else {
                if (mobius_mqtt_client) {
                    console.log('set Home_Position -', btn_params.x / 10000000, btn_params.y / 10000000, 'get Target Drone Position -', fc.global_position_int.lat, fc.global_position_int.lat);
                    mobius_mqtt_client.publish(pub_topic, msg);
                }
            }
        }
        catch (ex) {
            console.log('[ERROR] ' + ex);
        }
    }
}

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id, mavVersion);
    let mavMsg = null;
    let genMsg = null;
    try {
        //let targetSysId = sysId;
        let targetCompId = (params.targetCompId === undefined) ? 0 : params.targetCompId;

        switch (type) {
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
                    params.param7
                );
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
        }
    }
    catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
        //console.log('>>>>> MAVLINK OUTGOING MSG: ' + genMsg.toString('hex'));
    }

    return genMsg;
}