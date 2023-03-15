const { SerialPort } = require("serialport");
const SerialPortParser = require("@serialport/parser-readline");
const GPS = require("gps");
const mqtt = require('mqtt');
const mavlink = require('./mavlink.js');
const moment = require('moment');
const { nanoid } = require('nanoid');

let gpsPort = null;
// let gpsPortNum = 'COM32';
let gpsPortNum = '/dev/ttyAMA2';
let gpsBaudrate = '9600';
gpsPortOpening();

const gps = new GPS();
const parser = gpsPort.pipe(new SerialPortParser());

let globalpositionint_msg = '';
let gpsrawint_msg = '';
let heartbeat_msg = '';
let boot_start_time = 0;
let my_system_id = 254;

let local_mqtt_host = '127.0.0.1';
let mqtt_client = null;
let pub_gps_tracker_location_topic = '/GPS/location';

let mavData = {};
mavData.fix_type = 0;
mavData.lat = 0;
mavData.lon = 0;
mavData.alt = 0;
mavData.relative_alt = 0;
mavData.eph = 0;
mavData.epv = 0;
mavData.vel = 0;
mavData.cog = 0;
mavData.satellites_visible = 0;
mavData.vx = 0;
mavData.hdg = 0;

function gpsPortOpening() {
    if (gpsPort == null) {
        gpsPort = new SerialPort({
            path: gpsPortNum,
            baudRate: parseInt(gpsBaudrate, 10),
        });

        gpsPort.on('open', gpsPortOpen);
        gpsPort.on('close', gpsPortClose);
        gpsPort.on('error', gpsPortError);
        gpsPort.on('data', gpsPortData);
    } else {
        if (can_port.isOpen) {
            can_port.close();
            can_port = null;
            setTimeout(canPortOpening, 2000);
        } else {
            can_port.open();
        }
    }
}

function gpsPortOpen() {
    console.log('gpsPort open. ' + gpsPortNum + ' Data rate: ' + gpsBaudrate);
    mqtt_connect(local_mqtt_host);
}

function gpsPortClose() {
    console.log('gpsPort closed.');

    setTimeout(gpsPortOpening, 2000);
}

function gpsPortError(error) {
    console.log('[gpsPort error]: ' + error.message);

    setTimeout(gpsPortOpening, 2000);
}

function gpsPortData() {
    gps.on("data", data => {
        // console.log("gps data === ", data);
        if (data.type === 'GGA') {
            if (data.quality != null) {
                mavData.lat = data.lat;
                mavData.lon = data.lon;
                mavData.alt = data.alt;
                mavData.satellites_visible = data.satellites;
                mavData.eph = data.hdop;
                if (data.quality === 2) {
                    mavData.fix_type = 4;
                }
            } else {
                mavData.lat = 0;
                mavData.lon = 0;
                mavData.alt = 0;
                mavData.relative_alt = 0;
                mavData.eph = 0;
                mavData.satellites_visible = 0;
                mavData.fix_type = 1
            }
            setTimeout(createMAVLinkData, 1, my_system_id, boot_time, mavData);
        } else if (data.type === 'GSA') {
            if (mavData.fix_type !== 4) {
                if (data.fix === '3D') {
                    mavData.fix_type = 3;
                } else if (data.fix === '2D') {
                    mavData.fix_type = 2;
                } else {
                    mavData.fix_type = 1;
                }
            }
            mavData.eph = data.hdop;
            mavData.epv = data.vdop;
            setTimeout(createMAVLinkData, 1, my_system_id, boot_time, mavData);
        } else if (data.type === 'RMC') {
            mavData.vx = data.speed / 1.944;
            mavData.vel = data.speed / 1.944;
            mavData.cog = data.track;
            // console.log(data);
            setTimeout(createMAVLinkData, 1, my_system_id, boot_time, mavData);
        } else if (data.type === 'VTG') {
            mavData.vx = data.speed / 1.944;
            mavData.vel = data.speed / 1.944;
            mavData.hdg = data.track;
            // console.log(data);
            setTimeout(createMAVLinkData, 1, my_system_id, boot_time, mavData);
        }
        // #0, HEARTBEAT
        let params = {}
        params.target_system = my_system_id;
        params.target_component = 1;
        params.type = 6;
        params.autopilot = 8; // MAV_AUTOPILOT_INVALID
        params.base_mode = 128;
        params.custom_mode = 0;
        params.system_status = 3;
        params.mavlink_version = 1;

        try {
            heartbeat_msg = mavlinkGenerateMessage(params.target_system, params.target_component, mavlink.MAVLINK_MSG_ID_HEARTBEAT, params);
            if (heartbeat_msg == null) {
                console.log("mavlink message(MAVLINK_MSG_ID_HEARTBEAT) is null");
            } else {
                // console.log(heartbeat_msg)
            }
        } catch (ex) {
            console.log('[ERROR (HEARTBEAT)] ' + ex);
        }
        // send_aggr_to_Mobius(my_cnt_name, heartbeat_msg.toString('hex'), 1000);
        // mqtt_client.publish(gps_tracker_topic, Buffer.from(heartbeat_msg, 'hex'));
    });
}

parser.on("data", data => {
    // console.log('parser', data)
    gps.update(data);
});

function mqtt_connect(broker_ip) {
    if (mqtt_client == null) {
        let connectOptions = {
            host: broker_ip,
            port: 1883,
            protocol: "mqtt",
            keepalive: 10,
            clientId: 'GPS_' + nanoid(15),
            protocolId: "MQTT",
            protocolVersion: 4,
            clean: true,
            reconnectPeriod: 2000,
            connectTimeout: 2000,
            rejectUnauthorized: false
        }


        mqtt_client = mqtt.connect(connectOptions);

        mqtt_client.on('connect', function () {
            console.log('mqtt connected to ' + broker_ip);
        });

        mqtt_client.on('error', function (err) {
            console.log('[mqtt_client error] ' + err.message);
            setTimeout(mqtt_connect, 1000, broker_ip);
        });
    }
}

setInterval(function () {
    boot_time = moment().valueOf() - boot_start_time;
}, 1);

function createMAVLinkData(sys_id, boot_time, mavdata) {
    // #33, GLOBAL_POSITION_INT
    let params = {}
    params.target_system = sys_id;
    params.target_component = 1;
    params.time_boot_ms = boot_time;
    params.lat = parseFloat(mavdata.lat) * 1E7;
    params.lon = parseFloat(mavdata.lon) * 1E7;
    params.alt = parseFloat(mavdata.alt) * 1000;
    params.relative_alt = 0;
    params.vx = mavdata.vx;
    params.vy = 0;
    params.vz = 0;
    params.hdg = mavdata.hdg;

    try {
        globalpositionint_msg = mavlinkGenerateMessage(params.target_system, params.target_component, mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, params);
        if (globalpositionint_msg == null) {
            console.log("mavlink message(MAVLINK_MSG_ID_GLOBAL_POSITION_INT) is null");
        } else {
            // console.log(globalpositionint_msg)
        }
    } catch (ex) {
        console.log('[ERROR (GLOBAL_POSITION_INT)] ' + ex);
    }
    // mqtt_client.publish(my_cnt_name, Buffer.from(globalpositionint_msg, 'hex'));
    mqtt_client.publish(pub_gps_tracker_location_topic, Buffer.from(globalpositionint_msg, 'hex'));

    // #24, GPS_RAW_INT
    params = {}
    params.target_system = sys_id;
    params.target_component = 1;
    params.time_usec = boot_time;
    params.fix_type = mavdata.fix_type;
    params.lat = parseFloat(mavdata.lat) * 1E7;
    params.lon = parseFloat(mavdata.lon) * 1E7;
    params.alt = parseFloat(mavdata.alt) * 1000;
    params.eph = parseFloat(mavdata.eph) * 100;
    params.epv = parseFloat(mavdata.epv) * 100;
    params.vel = mavdata.vel;
    params.cog = mavdata.cog;
    params.satellites_visible = mavdata.satellites_visible;

    try {
        gpsrawint_msg = mavlinkGenerateMessage(params.target_system, params.target_component, mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, params);
        if (gpsrawint_msg == null) {
            console.log("mavlink message(MAVLINK_MSG_ID_GPS_RAW_INT) is null");
        } else {
            // console.log(gpsrawint_msg)
        }
    } catch (ex) {
        console.log('[ERROR (GPS_RAW_INT)] ' + ex);
    }
    // mqtt_client.publish(gps_tracker_topic, Buffer.from(gpsrawint_msg, 'hex'));
}

function mavlinkGenerateMessage(src_sys_id, src_comp_id, type, params) {
    const mavlinkParser = new MAVLink(null/*logger*/, src_sys_id, src_comp_id);
    try {
        var mavMsg = null;
        var genMsg = null;

        switch (type) {
            case mavlink.MAVLINK_MSG_ID_HEARTBEAT:
                mavMsg = new mavlink.messages.heartbeat(params.type,
                    params.autopilot,
                    params.base_mode,
                    params.custom_mode,
                    params.system_status,
                    params.mavlink_version
                );
                break;
            case mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                mavMsg = new mavlink.messages.global_position_int(params.time_boot_ms,
                    params.lat,
                    params.lon,
                    params.alt,
                    params.relative_alt,
                    params.vx,
                    params.vy,
                    params.vz,
                    params.hdg
                );
                break;
            case mavlink.MAVLINK_MSG_ID_GPS_RAW_INT:
                mavMsg = new mavlink.messages.gps_raw_int(params.time_usec,
                    params.fix_type,
                    params.lat,
                    params.lon,
                    params.alt,
                    params.eph,
                    params.epv,
                    params.vel,
                    params.cog,
                    params.satellites_visible,
                    params.alt_ellipsoid,
                    params.h_acc,
                    params.v_acc,
                    params.vel_acc,
                    params.hdg_acc
                );
                break;
        }
    } catch (e) {
        console.log('MAVLINK EX:' + e);
    }

    if (mavMsg) {
        genMsg = Buffer.from(mavMsg.pack(mavlinkParser));
        //console.log('>>>>> MAVLINK OUTGOING MSG: ' + genMsg.toString('hex'));
    }

    return genMsg;
}
