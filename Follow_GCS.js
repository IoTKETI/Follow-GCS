/**
 * Created by Wonseok Jung in KETI on 2023-03-25.
 */

require("moment-timezone");
const moment = require('moment');
moment.tz.setDefault("Asia/Seoul");
const mqtt = require("mqtt");
const {nanoid} = require("nanoid");
const fs = require("fs");

const mavlink = require('./mavlibrary/mavlink');

// TODO: gcs_position에 GCS 위치 연동
// const gcs_position = '37.40363759298978, 127.16174915851377';
const gcs_position = '37.40322371991492, 127.16346364704359';

let drone_info = {};

let fc = {
    "heartbeat": {},
    "global_position_int":{
        "lat": 0,
        "lon": 0,
        "relative_alt": 20
    }
};

let mqtt_client = null;
let drone_topic = '';
let cmd_topic = '';

let ardupilot_mode_indexs_obj = {
    'STABILIZE': '00000000',
    'ACRO': '01000000',
    'ALT_HOLD': '02000000',
    'AUTO': '03000000',
    'GUIDED': '04000000',
    'LOITER': '05000000',
    'RTL': '06000000',
    'CIRCLE': '07000000',
    'POSITION': '08000000',
    'LAND': '09000000',
    'OF_LOITER': '0a000000',
    'DRIFT': '0b000000',
    'RESERVED_12': '0c000000',
    'SPORT': '0d000000',
    'FLIP': '0e000000',
    'AUTOTUNE': '0f000000',
    'POS_HOLD': '10000000',
    'BRAKE': '11000000',
    'THROW': '12000000',
    'AVOID_ADSB': '13000000',
    'GUIDED_NOGPS': '14000000',
    'SAFE_RTL': '15000000'
}

init();

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
            connectTimeout: 2000,
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
            mqtt_client = null;
            mqtt_connect(serverip);
        })
    }
}

let mavVersion = 'v1';

function parseMavFromDrone(mavPacket) {
    try {
        let ver = mavPacket.substring(0, 2);
        // let sys_id = '';
        let msg_id;
        let base_offset;

        if (ver === 'fd') {
            // sys_id = parseInt(mavPacket.substring(10, 12).toLowerCase(), 16);
            msg_id = parseInt(mavPacket.substring(18, 20) + mavPacket.substring(16, 18) + mavPacket.substring(14, 16), 16);
            mavVersion = 'v2';
            base_offset = 20;
        } else {
            // sys_id = parseInt(mavPacket.substring(6, 8).toLowerCase(), 16);
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
            if (fc.heartbeat.type !== mavlink.MAV_TYPE_ADSB) {
                fc.heartbeat.autopilot = Buffer.from(autopilot, 'hex').readUInt8(0);
                fc.heartbeat.base_mode = Buffer.from(base_mode, 'hex').readUInt8(0);
                fc.heartbeat.custom_mode = Buffer.from(custom_mode, 'hex').readUInt32LE(0);
                fc.heartbeat.system_status = Buffer.from(system_status, 'hex').readUInt8(0);
                fc.heartbeat.mavlink_version = Buffer.from(mavlink_version, 'hex').readUInt8(0);
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

            fc.global_position_int = {};
            fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0) / 10000000;
            fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0) / 10000000;
            fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0) / 1000;
            fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0) / 1000;
        }
    } catch
        (e) {
        console.log('[parseMavFromDrone Error]', e);
    }
}

function init() {
    try {
        drone_info = JSON.parse(fs.readFileSync('../drone_info.json', 'utf8'));
    } catch (e) {
        console.log('can not find [ ../drone_info.json ] file');
        drone_info.host = "121.147.228.240";
        drone_info.drone = "KETI_Simul_1";
        drone_info.gcs = "KETI_GCS";
        drone_info.type = "ardupilot";
        drone_info.system_id = 105;
        drone_info.kcmvp = "off";
        drone_info.id = "UA";
        // fs.writeFileSync('../drone_info.json', JSON.stringify(drone_info, null, 4), 'utf8');
    }

    drone_topic = '/Mobius/' + drone_info.gcs + '/Drone_Data/' + drone_info.drone;
    cmd_topic = '/Mobius/' + drone_info.gcs + '/GCS_Data/' + drone_info.drone;
    mqtt_connect(drone_info.host, drone_topic);

    setInterval(goto_command, 1000);
}

function goto_command() {

    // set GUIDED Mode
    let target_mode = 'GUIDED';

    setTimeout(send_set_mode_command, 10, cmd_topic, drone_info.system_id, target_mode);

    let gcs_position_arr = gcs_position.split(', ');
    let lat = parseFloat(gcs_position_arr[0]);
    let lon = parseFloat(gcs_position_arr[1]);
    let alt = parseFloat(fc.global_position_int.relative_alt);
    // let speed = parseFloat(arr_cur_goto_position[3]);
    let speed = 5.0;
    console.log('send_goto_command', lat, lon, alt, speed);


    setTimeout(send_goto_command, 50 + 20, cmd_topic, drone_info.system_id, lat, lon, alt);

    setTimeout(send_change_speed_command, 500 + 50, cmd_topic, drone_info.system_id, speed);
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
                mavMsg = new mavlink.messages.command_long(params.target_system,
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
                mavMsg = new mavlink.messages.mission_item(params.target_system,
                    params.target_component,
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
                    params.param7);
                break;
            case mavlink.MAVLINK_MSG_ID_SET_MODE:
                mavMsg = new mavlink.messages.set_mode(params.target_system,
                    params.base_mode,
                    params.custom_mode);
                break;
            case mavlink.MAVLINK_MSG_ID_MISSION_ITEM:
                mavMsg = new mavlink.messages.mission_item(params.target_system,
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
                break;}
    }
    catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
        // console.log('>>>>> MAVLINK OUTGOING MSG: ' + genMsg.toString('hex'));
    }

    return genMsg;
}

function send_set_mode_command(pub_topic, target_sys_id, target_mode) {
    var str_custom_mode = ardupilot_mode_indexs_obj[target_mode];
    var custom_mode = Buffer.from(str_custom_mode, 'hex').readUInt32LE(0);
    var base_mode = fc.heartbeat.base_mode & ~mavlink.MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE;
    base_mode |= mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

    var btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.base_mode = base_mode;
    btn_params.custom_mode = custom_mode;

    try {
        var msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_SET_MODE, btn_params);
        if (msg === null) {
            console.log("[send_set_mode_command] mavlink message is null");
        } else {
            mqtt_client.publish(pub_topic, msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function send_goto_command(pub_topic, target_sys_id, latitude, longitude, rel_altitude) {
    var btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.target_component = 1;
    btn_params.seq = 0;
    btn_params.frame = mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT; // 0: MAV_FRAME_GLOBAL, 3: MAV_FRAME_GLOBAL_RELATIVE_ALT
    btn_params.command = mavlink.MAV_CMD_NAV_WAYPOINT;
    btn_params.current = 2;
    btn_params.autocontinue = 0;
    btn_params.param1 = 0;
    btn_params.param2 = 0;
    btn_params.param3 = 0;
    btn_params.param4 = 0;
    btn_params.param5 = latitude;
    btn_params.param6 = longitude;
    btn_params.param7 = rel_altitude;
    btn_params.mission_type = 0;

    //console.log(latitude, longitude, rel_altitude);

    try {
        var msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_MISSION_ITEM, btn_params);
        if (msg == null) {
            console.log("[send_goto_command] mavlink message is null");
        } else {
            mqtt_client.publish(pub_topic, msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function send_change_speed_command(pub_topic, target_sys_id, target_speed) {
    var btn_params = {};
    btn_params.target_system = target_sys_id;
    btn_params.target_component = 1;
    btn_params.command = mavlink.MAV_CMD_DO_CHANGE_SPEED;
    btn_params.confirmation = 0;
    btn_params.param1 = 0; // Speed type (0=Airspeed, 1=Ground Speed).
    btn_params.param2 = target_speed; // Target speed (m/s).
    btn_params.param3 = 0; // Throttle as a percentage (0-100%). A value of -1 indicates no change.
    btn_params.param4 = 0; // Empty
    btn_params.param5 = 0; // Empty
    btn_params.param6 = 0; // Empty
    btn_params.param7 = 0; // Empty

    try {
        var msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_COMMAND_LONG, btn_params);
        if (msg == null) {
            console.log("[send_change_speed_command] mavlink message is null");
        } else {
            mqtt_client.publish(pub_topic, msg);
        }
    } catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}
