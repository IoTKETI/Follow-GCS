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

let target_position = '0, 0, 0';
let dist = 0;
let tracking = false;
let cur_track = false;

const dist_threshold = 20.0;

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
let target_topic = '';

let cur_mode = '';

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

let ardupilot_mode_items_obj = {
    '00000000': 'STABILIZE',
    '01000000': 'ACRO',
    '02000000': 'ALT_HOLD',
    '03000000': 'AUTO',
    '04000000': 'GUIDED',
    '05000000': 'LOITER',
    '06000000': 'RTL',
    '07000000': 'CIRCLE',
    '08000000': 'POSITION',
    '09000000': 'LAND',
    '0a000000': 'OF_LOITER',
    '0b000000': 'DRIFT',
    '0c000000': 'RESERVED_12',
    '0d000000': 'SPORT',
    '0e000000': 'FLIP',
    '0f000000': 'AUTOTUNE',
    '10000000': 'POS_HOLD',
    '11000000': 'BRAKE',
    '12000000': 'THROW',
    '13000000': 'AVOID_ADSB',
    '14000000': 'GUIDED_NOGPS',
    '15000000': 'SAFE_RTL'
}

init();

function mobius_mqtt_connect(serverip, d_topic, t_topic) {
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
            console.log('mqtt is connected to (' + serverip + ')');

            if (d_topic !== '') {  // GCS topic
                mobius_mqtt_client.subscribe(d_topic + '/#', () => {
                    console.log('[mobius_mqtt_connect] drone_topic is subscribed: ' + d_topic + '/#');
                });
            }
            if (t_topic !== '') {  // GCS topic
                mobius_mqtt_client.subscribe(t_topic + '/#', () => {
                    console.log('[mobius_mqtt_connect] target_topic is subscribed: ' + t_topic + '/#');
                });
            }
        });

        mobius_mqtt_client.on('message', (topic, message) => {
            if (topic.includes(d_topic)) {
                let _msg = message.toString('hex');
                parseMavFromDrone(_msg);
            }
            else if (topic.includes(t_topic)) {
                let _msg = message.toString('hex');
                parseMavFromDrone(_msg);
            }
        })

        mobius_mqtt_client.on('error', (err) => {
            console.log('[mqtt] (error) ' + err.message);
        })
    }
}

let mavVersion = 'v1';
let target_lat;
let target_lon;
let target_alt;

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
            let my_len = 9;
            let ar = mavPacket.split('');
            for (let i = 0; i < (my_len - msg_len); i++) {
                ar.splice(ar.length-4, 0, '0');
                ar.splice(ar.length-4, 0, '0');
            }
            mavPacket = ar.join('');

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

                cur_mode = ardupilot_mode_items_obj[custom_mode];
            }
        }
        else if (msg_id === mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // #33
            let my_len = 28;
            let ar = mavPacket.split('');
            for (let i = 0; i < (my_len - msg_len); i++) {
                ar.splice(ar.length - 4, 0, '0');
                ar.splice(ar.length - 4, 0, '0');
            }
            mavPacket = ar.join('');

            // var time_boot_ms = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let lat = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let lon = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();
            base_offset += 8;
            let relative_alt = mavPacket.substring(base_offset, base_offset + 8).toLowerCase();

            if (sys_id === parseInt(drone_info.system_id)) {
                fc.global_position_int = {};
                fc.global_position_int.lat = Buffer.from(lat, 'hex').readInt32LE(0) / 10000000;
                fc.global_position_int.lon = Buffer.from(lon, 'hex').readInt32LE(0) / 10000000;
                fc.global_position_int.alt = Buffer.from(alt, 'hex').readInt32LE(0) / 1000;
                fc.global_position_int.relative_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0) / 1000;
            }
            else {
                target_lat = Buffer.from(lat, 'hex').readInt32LE(0) / 10000000;
                target_lon = Buffer.from(lon, 'hex').readInt32LE(0) / 10000000;
                target_alt = Buffer.from(relative_alt, 'hex').readInt32LE(0) / 1000;
                target_position = target_lat + ', ' + target_lon + ', ' + target_alt;
                // console.log(sys_id + ' gcs_position: ' + target_position);
            }

            let dist = calcDistance(fc.global_position_int.lat, fc.global_position_int.lon, 0, target_lat, target_lon, 0);
            console.log('dist:', dist);

            if (dist > dist_threshold) {
                tracking = true;
                cur_track = true;
            }
            else {
                // dist_threshold 만큼 가까워지면 멈추도록 명령
                tracking = false;
                if (cur_track) {
                    let target_mode = 'LOITER';
                    setTimeout(send_set_mode_command, 10, cmd_topic, drone_info.system_id, target_mode);
                    target_mode = 'GUIDED';
                    setTimeout(send_set_mode_command, 10, cmd_topic, drone_info.system_id, target_mode);
                    cur_track = false;
                }
            }
        }
    }
    catch
        (e) {
        console.log('[parseMavFromDrone Error]', e);
    }
}

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

    target_topic = '/Mobius/' + drone_info.follow_target.gcs + '/Drone_Data/' + drone_info.follow_target.drone;
    mobius_mqtt_connect(drone_info.host, drone_topic, target_topic);

    setInterval(goto_command, 5000);
}

function goto_command() {
    if (cur_track) {
        if (cur_mode !== 'GUIDED') {
            // set GUIDED Mode
            let target_mode = 'GUIDED';

            setTimeout(send_set_mode_command, 10, cmd_topic, drone_info.system_id, target_mode);
        }
        let gcs_position_arr = target_position.split(', ');
        let target_lat = parseFloat(gcs_position_arr[0]);
        let target_lon = parseFloat(gcs_position_arr[1]);
        let target_alt = parseFloat(gcs_position_arr[2]);
        // let speed = parseFloat(arr_cur_goto_position[3]);
        let speed = 5.0;

        if (target_lat !== 0.0 && target_lon !== 0.0 && tracking) {
            setTimeout(send_goto_command, 50 + 20, cmd_topic, drone_info.system_id, target_lat, target_lon, target_alt);

            setTimeout(send_change_speed_command, 500 + 50, cmd_topic, drone_info.system_id, speed);
        }
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
                break;
        }
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
        let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_SET_MODE, btn_params);
        if (!msg) {
            console.log("[send_set_mode_command] mavlink message is null");
        }
        else {
            mobius_mqtt_client.publish(pub_topic, msg);
        }
    }
    catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function send_goto_command(pub_topic, target_sys_id, latitude, longitude, rel_altitude) {
    let btn_params = {};
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
        let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_MISSION_ITEM, btn_params);
        if (!msg) {
            console.log("[send_goto_command] mavlink message is null");
        }
        else {
            mobius_mqtt_client.publish(pub_topic, msg);
        }
    }
    catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

function send_change_speed_command(pub_topic, target_sys_id, target_speed) {
    let btn_params = {};
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
        let msg = mavlinkGenerateMessage(255, 0xbe, mavlink.MAVLINK_MSG_ID_COMMAND_LONG, btn_params);
        if (!msg) {
            console.log("[send_change_speed_command] mavlink message is null");
        }
        else {
            mobius_mqtt_client.publish(pub_topic, msg);
        }
    }
    catch (ex) {
        console.log('[ERROR] ' + ex);
    }
}

// const RE = 6371.00877; // 지구 반경(km)
// const GRID = 0.001; // 격자 간격(km)
// const SLAT1 = 30.0; // 투영 위도1(degree)
// const SLAT2 = 60.0; // 투영 위도2(degree)
// const OLON = 126.0; // 기준점 경도(degree)
// const OLAT = 38.0; // 기준점 위도(degree)
// const XO = 43; // 기준점 X좌표(GRID)
// const YO = 136; // 기1준점 Y좌표(GRID)
//
// function dfs_xy_conv(code, v1, v2) {
//     const DEGRAD = Math.PI / 180.0;
//     const RADDEG = 180.0 / Math.PI;
//
//     const re = RE / GRID;
//     const slat1 = SLAT1 * DEGRAD;
//     const slat2 = SLAT2 * DEGRAD;
//     const olon = OLON * DEGRAD;
//     const olat = OLAT * DEGRAD;
//
//     let sn = Math.tan(Math.PI * 0.25 + slat2 * 0.5) / Math.tan(Math.PI * 0.25 + slat1 * 0.5);
//     sn = Math.log(Math.cos(slat1) / Math.cos(slat2)) / Math.log(sn);
//     let sf = Math.tan(Math.PI * 0.25 + slat1 * 0.5);
//     sf = Math.pow(sf, sn) * Math.cos(slat1) / sn;
//     var ro = Math.tan(Math.PI * 0.25 + olat * 0.5);
//     ro = re * sf / Math.pow(ro, sn);
//     let rs = {};
//     if (code === "toXY") {
//         rs['lat'] = v1;
//         rs['lng'] = v2;
//         var ra = Math.tan(Math.PI * 0.25 + (v1) * DEGRAD * 0.5);
//         ra = re * sf / Math.pow(ra, sn);
//         var theta = v2 * DEGRAD - olon;
//         if (theta > Math.PI) theta -= 2.0 * Math.PI;
//         if (theta < -Math.PI) theta += 2.0 * Math.PI;
//         theta *= sn;
//         rs['x'] = Math.floor(ra * Math.sin(theta) + XO + 0.5);
//         rs['y'] = Math.floor(ro - ra * Math.cos(theta) + YO + 0.5);
//     } else {
//         rs['x'] = v1;
//         rs['y'] = v2;
//         let xn = v1 - XO;
//         let yn = ro - v2 + YO;
//         ra = Math.sqrt(xn * xn + yn * yn);
//         if (sn < 0.0) -ra;
//         let alat = Math.pow((re * sf / ra), (1.0 / sn));
//         alat = 2.0 * Math.atan(alat) - Math.PI * 0.5;
//
//         if (Math.abs(xn) <= 0.0) {
//             theta = 0.0;
//         } else {
//             if (Math.abs(yn) <= 0.0) {
//                 theta = Math.PI * 0.5;
//                 if (xn < 0.0) -theta;
//             } else theta = Math.atan2(xn, yn);
//         }
//         let alon = theta / sn + olon;
//         rs['lat'] = alat * RADDEG;
//         rs['lng'] = alon * RADDEG;
//     }
//     return rs;
// }

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

