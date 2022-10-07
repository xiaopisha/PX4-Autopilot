/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef ADSB_VEHICLE_HPP
#define ADSB_VEHICLE_HPP

#include <uORB/topics/transponder_report.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

class MavlinkStreamADSBVehicle : public ModuleParams, public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamADSBVehicle(mavlink); }

	static constexpr const char *get_name_static() { return "ADSB_VEHICLE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_ADSB_VEHICLE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	bool const_rate() override { return true; }

	unsigned get_size() override
	{
		return _transponder_report_sub.advertised() ? MAVLINK_MSG_ID_ADSB_VEHICLE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamADSBVehicle(Mavlink *mavlink) : ModuleParams(nullptr), MavlinkStream(mavlink) {}

	uORB::Subscription _transponder_report_sub{ORB_ID(transponder_report)};

#if CONFIG_DRIVERS_TRANSPONDER_UAVIONIX

	uORB::Subscription _vehicle_gps_position_sub{ORB_ID(vehicle_gps_position)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Required update for cfg (static) msg is 0.1 [Hz] ->  5 [Hz] / 0.1 [Hz] = 50
	uint8_t _period_counter = CFG_RESET_COUNTER;
	static constexpr uint8_t CFG_PERIOD_COUNTER = 50;
	static constexpr uint8_t CFG_RESET_COUNTER = 0;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ADSB_SQUAWK>)		_adsb_squawk,
		(ParamInt<px4::params::ADSB_ICAO_ID>)		_adsb_icao,
		(ParamInt<px4::params::ADSB_LEN_WIDTH>)		_adsb_len_width,
		(ParamInt<px4::params::ADSB_EMIT_TYPE>)		_adsb_emit_type,
		(ParamInt<px4::params::ADSB_EMERGC>)		_adsb_emergc
	);

#endif

	bool send() override
	{
		bool sent = false;

		transponder_report_s pos;

		while ((_mavlink->get_free_tx_buf() >= get_size()) && _transponder_report_sub.update(&pos)) {

			if (!(pos.flags & transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE)) {
				continue;
			}

			mavlink_adsb_vehicle_t msg{};
			msg.ICAO_address = pos.icao_address;
			msg.lat = pos.lat * 1e7;
			msg.lon = pos.lon * 1e7;
			msg.altitude_type = pos.altitude_type;
			msg.altitude = pos.altitude * 1e3f;
			msg.heading = (pos.heading + M_PI_F) / M_PI_F * 180.0f * 100.0f;
			msg.hor_velocity = pos.hor_velocity * 100.0f;
			msg.ver_velocity = pos.ver_velocity * 100.0f;
			memcpy(&msg.callsign[0], &pos.callsign[0], sizeof(msg.callsign));
			msg.emitter_type = pos.emitter_type;
			msg.tslc = pos.tslc;
			msg.squawk = pos.squawk;

			msg.flags = 0;

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS) { msg.flags |= ADSB_FLAGS_VALID_COORDS; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE) { msg.flags |= ADSB_FLAGS_VALID_ALTITUDE; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING) { msg.flags |= ADSB_FLAGS_VALID_HEADING; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY) { msg.flags |= ADSB_FLAGS_VALID_VELOCITY; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN) { msg.flags |= ADSB_FLAGS_VALID_CALLSIGN; }

			if (pos.flags & transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK) { msg.flags |= ADSB_FLAGS_VALID_SQUAWK; }

			mavlink_msg_adsb_vehicle_send_struct(_mavlink->get_channel(), &msg);
			sent = true;
		}

#if CONFIG_DRIVERS_TRANSPONDER_UAVIONIX

		if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_UAVIONIX) {

			// Required update for dynamic message is 5 [Hz]
			mavlink_uavionix_adsb_out_dynamic_t dynamic_msg = {
				.utcTime = UINT32_MAX,
				.gpsLat = INT32_MAX,
				.gpsLon = INT32_MAX,
				.gpsAlt = INT32_MAX,
				.baroAltMSL = INT32_MAX,
				.accuracyHor = UINT32_MAX,
				.accuracyVert = UINT16_MAX,
				.accuracyVel = UINT16_MAX,
				.velVert = INT16_MAX,
				.velNS = INT16_MAX,
				.VelEW = INT16_MAX,
				.state = UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND,
				.squawk = static_cast<uint16_t>(_adsb_squawk.get()),
				.gpsFix = 0,
				.numSats = UINT8_MAX,
				.emergencyStatus = static_cast<uint8_t>(_adsb_emergc.get())
			};

			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.update(&vehicle_status)) {
				if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
					dynamic_msg.state |= ~UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
				}
			}

			sensor_gps_s vehicle_gps_position;

			if (_vehicle_gps_position_sub.update(&vehicle_gps_position)) {

				dynamic_msg.utcTime = static_cast<uint32_t>(vehicle_gps_position.time_utc_usec / 1000000ULL);
				dynamic_msg.gpsLat = vehicle_gps_position.lat;
				dynamic_msg.gpsLon = vehicle_gps_position.lon;
				dynamic_msg.gpsAlt = vehicle_gps_position.alt;
				dynamic_msg.gpsFix = vehicle_gps_position.fix_type;
				dynamic_msg.numSats = vehicle_gps_position.satellites_used;
				dynamic_msg.accuracyHor = static_cast<uint32_t>(vehicle_gps_position.eph * 1000.0f); // convert [m] to [mm]
				dynamic_msg.accuracyVert = static_cast<uint16_t>(vehicle_gps_position.epv * 100.0f); // convert [m] to [cm]
				//_dynamic_data.accuracyVel //TODO: anything to set here?
				dynamic_msg.velVert = static_cast<int16_t>(vehicle_gps_position.vel_d_m_s * 100.0f); // convert [m/s] to [cm/s]
				dynamic_msg.velNS = static_cast<int16_t>(vehicle_gps_position.vel_n_m_s * 100.0f); // convert [m/s] to [cm/s]
				dynamic_msg.VelEW = static_cast<int16_t>(vehicle_gps_position.vel_e_m_s * 100.0f); // convert [m/s] to [cm/s]
			}

			vehicle_air_data_s vehicle_air_data;

			if (_vehicle_air_data_sub.update(&vehicle_air_data)) {
				dynamic_msg.baroAltMSL = static_cast<int32_t>(vehicle_air_data.baro_pressure_pa / 100.0f); // convert [Pa] to [mBar]
			}

			mavlink_msg_uavionix_adsb_out_dynamic_send_struct(_mavlink->get_channel(), &dynamic_msg);

			_period_counter++;

			if (_period_counter >= CFG_PERIOD_COUNTER) {

				// Required update for static message is 0.1 [Hz]
				mavlink_uavionix_adsb_out_cfg_t cfg_msg = {
					.ICAO = static_cast<uint32_t>(_adsb_icao.get()),
					.stallSpeed = 0, //TODO: any existing param?
					//memcpy(cfg_msg.callsign, "PX4 UAV ", sizeof(cfg_msg.callsign)); TODO: shall to be changeable
					.emitterType = static_cast<uint8_t>(_adsb_emit_type.get()),
					.aircraftSize = static_cast<uint8_t>(_adsb_len_width.get()),
					.gpsOffsetLat = 0x0, //TODO: add missing param
					.gpsOffsetLon = 0x0, //TODO: add missing param
					.rfSelect = UAVIONIX_ADSB_OUT_RF_SELECT_TX_ENABLED //TODO: add enum as param
				};

				mavlink_msg_uavionix_adsb_out_cfg_send_struct(_mavlink->get_channel(), &cfg_msg);
				_period_counter = CFG_RESET_COUNTER;
			}

			sent = true;
		}

#endif

		return sent;
	}
};

#endif // ADSB_VEHICLE_HPP
