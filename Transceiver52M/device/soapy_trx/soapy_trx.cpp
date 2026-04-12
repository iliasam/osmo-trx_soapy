/*
* Copyright 2018 sysmocom - s.f.m.c. GmbH
*
* SPDX-License-Identifier: AGPL-3.0+
*
	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU Affero General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU Affero General Public License for more details.

	You should have received a copy of the GNU Affero General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include <map>

#include "Logger.h"
#include "Threads.h"
#include "Utils.h"
#include <chrono>

#include "soapy_trx.h"

extern "C" {
#include "trx_vty.h"
#include "osmo_signal.h"
#include <osmocom/core/utils.h>
}

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

// This test is needed to check how RX is hearing TX.
// RX and TX set to a same frequency (downling), TX is sending short sine pulses periodically (pulse start is at the center of the packet)
// RX data is processed to find start of the pulse. Using this it is possible to adjust 
// "SOAPY_TX_OFFSET_SAMPLES" and "SOAPY_TX_OFFSET_PACKETS" below
//#define SOAPY_LOOPBACK_TEST 

#define SOAPY_TX_OFFSET_SAMPLES		65
#define SOAPY_TX_OFFSET_PACKETS		5

// This value is fixed
#define SOAPY_RX_GAIN_DB			50

//This value can be overwritten
#define SOAPY_TX_GAIN_DB			60

#define SAMPLE_BUF_SZ    (1 << 20) /* Size of Rx timestamp based Ring buffer, in bytes */


#define SOAPY_PACKET_SIZE_SAMPLES     2500

//~1.08 MHz
#define SAMPLE_RATE_HZ              (GSMRATE * tx_sps)


#define DEV_HW_TYPE                 NORMAL
//#define DEV_HW_TYPE                 RESAMP_SOAPY1
	//return RESAMP_SOAPY1; //<<<<<<< Using RadioInterfaceResamp
	//return RESAMP_64M;

/* Device parameter descriptor */
struct dev_desc {
	/* Sample rate coef (without having TX/RX samples per symbol into account) */
	double rate;
	/* Sample rate coef (without having TX/RX samples per symbol into account), if multi-arfcn is enabled */
	double rate_multiarfcn;

	/* Device Name Prefix*/
	std::string name_prefix;
};

static const std::map<enum soapy_dev_type, struct dev_desc> dev_param_map {
	//{ SOAPY_TYPE1,   { true,  true,  GSMRATE, MCBTS_SPACING, 8.9e-5, 7.9e-5, "SOAPY DEV" } },
	{ SOAPY_TYPE1,   { GSMRATE, MCBTS_SPACING, "SOAPY DEV" } },
};

typedef std::tuple<soapy_dev_type, enum gsm_band> dev_band_key;
typedef std::map<dev_band_key, dev_band_desc>::const_iterator dev_band_map_it;

// 1 - TX max dB, 2 - TX max dBm, 3 - RX offset
static const std::map<dev_band_key, dev_band_desc> dev_band_nom_power_param_map {
	{ std::make_tuple(SOAPY_TYPE1, GSM_BAND_900),	{ 70.0, 3.0,  -6.0  } },
};

// We simply adjust the measured offset
static double TxGain2TxPower(const dev_band_desc &desc, double tx_gain_db)
{
	return desc.nom_out_tx_power - (desc.nom_sdr_tx_gain - tx_gain_db);
}
static double TxPower2TxGain(const dev_band_desc &desc, double tx_power_dbm)
{
	return desc.nom_sdr_tx_gain - (desc.nom_out_tx_power - tx_power_dbm);
}


soapy_device::soapy_device(size_t tx_sps, size_t rx_sps, InterfaceType iface, size_t chan_num, double lo_offset,
		     const std::vector<std::string>& tx_paths,
		     const std::vector<std::string>& rx_paths):
		     RadioDevice(tx_sps, rx_sps, iface, chan_num, lo_offset, tx_paths, rx_paths),
		     //m_lms_dev(NULL), started(false), band_ass_curr_sess(false), band((enum gsm_band)0),
		     started(false), band_ass_curr_sess(false), band((enum gsm_band)0),
		     m_dev_type(SOAPY_TYPE1)
{
	LOGC(DDEV, NOTICE) << "SOAPY TRX HW Init";

	rx_gains.resize(chans);
	tx_gains.resize(chans);

	if (chans != 1)
	{
		LOGC(DDEV, ERROR) << "Wrong number of channels!";
	}

	rx_buffers.resize(chans);

	/* Set up per-channel Rx timestamp based Ring buffers */
	rx_buffers[0] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

}

soapy_device::~soapy_device()
{
	LOGC(DDEV, NOTICE) << "Closing SOAPY device";

    if (device) {
		device->deactivateStream(txStream);
        device->closeStream(txStream);
		device->deactivateStream(rxStream);
        device->closeStream(rxStream);
		SoapySDR::Device::unmake(device);
	}

#ifdef SOAPY_LOOPBACK_TEST
	free(test_tx_buf);
#endif

	delete rx_buffers[0];
}

void soapy_device::assign_band_desc(enum gsm_band req_band)
{
	dev_band_map_it it;

	it = dev_band_nom_power_param_map.find(dev_band_key(m_dev_type, req_band));
	if (it == dev_band_nom_power_param_map.end()) {
		dev_desc desc = dev_param_map.at(m_dev_type);
		LOGC(DDEV, ERROR) << "No Tx Power measurements exist for device "
				    << desc.name_prefix << " on band " << gsm_band_name(req_band)
				    << ", using LimeSDR-USB ones as fallback";
		it = dev_band_nom_power_param_map.find(dev_band_key(SOAPY_TYPE1, req_band));
	}
	OSMO_ASSERT(it != dev_band_nom_power_param_map.end());
	band_desc = it->second;
}

bool soapy_device::set_band(enum gsm_band req_band)
{
	if (band_ass_curr_sess && req_band != band) {
		LOGC(DDEV, ALERT) << "Requesting band " << gsm_band_name(req_band)
				  << " different from previous band " << gsm_band_name(band);
		return false;
	}

	if (req_band != band) {
		band = req_band;
		assign_band_desc(band);
	}
	band_ass_curr_sess = true;
	return true;
}

void soapy_device::get_dev_band_desc(dev_band_desc& desc)
{
	if (band == 0) {
		LOGC(DDEV, ERROR) << "Power parameters requested before Tx Frequency was set! Providing band 900 by default...";
		assign_band_desc(GSM_BAND_900);
	}
	desc = band_desc;
}

int soapy_device::open(const std::string &args, int ref, bool swap_channels)
{
	LOGC(DDEV, INFO) << "SOAPY TRX HW Open";
	LOGC(DDEV, NOTICE) << args;
	const char* deviceArgs = args.c_str();
	m_dev_type = soapy_dev_type::SOAPY_TYPE1;

	SoapySDR::KwargsList sdr_results = SoapySDR::Device::enumerate("driver=plutosdr");

	if (sdr_results.empty()) 
	{
    	LOGC(DDEV, ERROR) << "Soapy device was not found!";
		return -1;
	}

	device = SoapySDR::Device::make(deviceArgs);

	if (device == nullptr) {
    	LOGC(DDEV, ERROR) << "Soapy device opening error!";
		return -1;
	}	

	ts_initial = 0;//do not delete!

	rx_timestamp_ns = 0;
	rx_timestamp_in_samples = 0;

	set_rates_tx();
	set_rates_rx();
	init_gains();

	txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, {0});
	rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, {0});

	if (txStream == nullptr)
	{
		LOGC(DDEV, ERROR) << "Soapy TX stream opening error!";
		return -1;
	}

	if (rxStream == nullptr)
	{
		LOGC(DDEV, ERROR) << "Soapy RX stream opening error!";
		return -1;
	}

	size_t tx_mtu_size = device->getStreamMTU(txStream);
	LOGC(DDEV, NOTICE) << "TX MTU Size " <<  tx_mtu_size;

	size_t rx_mtu_size = device->getStreamMTU(rxStream);
	LOGC(DDEV, NOTICE) << "RX MTU Size " <<  rx_mtu_size;
	
	int res = 0;
	res = device->activateStream(txStream); // Start the stream
	if (res != 0)
	{
		LOGC(DDEV, ERROR) << "Soapy TX stream activate error! " << res;
		return -1;
	}

	res = device->activateStream(rxStream); // Start the stream
	if (res != 0)
	{
		LOGC(DDEV, ERROR) << "Soapy RX stream activate error! " << res;
		return -1;
	}

	/* configure antennas */
	if (!set_antennas()) {
		LOGC(DDEV, FATAL) << "Soapy antenna setting failed";
	}

	//test_rx();

#ifdef SOAPY_LOOPBACK_TEST
	//iq buffer
	test_tx_buf = (uint16_t*) malloc(SOAPY_PACKET_SIZE_SAMPLES * sizeof(uint16_t) * 2);
	memset(test_tx_buf, 0, (SOAPY_PACKET_SIZE_SAMPLES * sizeof(uint16_t) * 2));
#endif

	LOGC(DDEV, NOTICE) << "creating SOAPY TRX device:"
			  << " RXSPS: " << rx_sps
			  << " TXSPS: " << tx_sps
			  << " chans: " << chans;

	return DEV_HW_TYPE;
}

void soapy_device::test_rx()
{
	int flags;//flags set by receive operation
	long long timeNs; //timestamp for receive buffer
	uint16_t tmp_rx_buf[SOAPY_PACKET_SIZE_SAMPLES * 2];//iq
	void *buf_rx[] = {tmp_rx_buf};

	device->setFrequency(SOAPY_SDR_RX, 0, (double)900e6);

	uint64_t diff_ns_prev = 0;
	uint64_t diff_sps_prev = 0;

	for (int i = 0; i < 20; i++)
	{
		thread_enable_cancel(false);
		device->readStream(rxStream, buf_rx, SOAPY_PACKET_SIZE_SAMPLES, flags, timeNs, 50000); // 50ms timeout
		uint64_t rx_timestamp_samples = SoapySDR::timeNsToTicks(timeNs, (double)SAMPLE_RATE_HZ);

		uint64_t diff_ns = timeNs - diff_ns_prev;
		diff_ns_prev = timeNs;

		uint64_t diff_sps = rx_timestamp_samples - diff_sps_prev;
		diff_sps_prev = rx_timestamp_samples;

		uint64_t rx_diff_samples = SoapySDR::timeNsToTicks(diff_ns, (double)SAMPLE_RATE_HZ);

		if (diff_sps != SOAPY_PACKET_SIZE_SAMPLES)
		{
			LOGC(DDEV, NOTICE) << "WRONG NUMBER SPS: " <<  diff_sps;
		}

		LOGC(DDEV, NOTICE) << " TIME diff: " << diff_ns << " time_ns: " << timeNs << " sps: " << rx_timestamp_samples 
			<< " sps diff: " << diff_sps << " diff2: " << rx_diff_samples << std::endl;
		thread_enable_cancel(true);
	}
}


bool soapy_device::start()
{
	LOGC(DDEV, INFO) << "Starting SOAPY...";

	if (started) {
		LOGC(DDEV, ERROR) << "Device already started";
		return false;
	}

	started = true;
	return true;
}

bool soapy_device::stop()
{
	LOGC(DDEV, NOTICE) << "dev stop";

	if (!started)
		return true;

	band_ass_curr_sess = false;
	started = false;
	LOGC(DDEV, NOTICE) << "dev stop done";

	return true;
}


double soapy_device::maxRxGain()
{
	return 73.0;
}

double soapy_device::minRxGain()
{
	return 0.0;
}

double soapy_device::setRxGain(double dB, size_t chan)
{
	if (dB > maxRxGain())
		dB = maxRxGain();
	if (dB < minRxGain())
		dB = minRxGain();

	//LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

	//if (LMS_SetGaindB(m_lms_dev, LMS_CH_RX, chan, dB) < 0)
	//	LOGCHAN(chan, DDEV, ERR) << "Error setting RX gain to " << dB << " dB";
	//else
		rx_gains[chan] = dB;
	return rx_gains[chan];
}

double soapy_device::rssiOffset(size_t chan)
{
	double rssiOffset;
	dev_band_desc desc;

	if (chan >= rx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	rssiOffset = rx_gains[chan] + desc.rxgain2rssioffset_rel;
	return rssiOffset;
}

double soapy_device::setPowerAttenuation(int atten, size_t chan)
{
	double tx_power, dB;
	dev_band_desc desc;

	if (chan >= tx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	tx_power = desc.nom_out_tx_power - atten;
	dB = TxPower2TxGain(desc, tx_power);

	LOGCHAN(chan, DDEV, NOTICE) << "Setting TX gain to " << dB << " dB (~" << tx_power << " dBm)";

	device->setGain(SOAPY_SDR_TX, 0, (double)dB);
	tx_gains[chan] = dB;
	return desc.nom_out_tx_power - TxGain2TxPower(desc, tx_gains[chan]);
}

double soapy_device::getPowerAttenuation(size_t chan) {
	dev_band_desc desc;
	if (chan >= tx_gains.size()) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return 0.0f;
	}

	get_dev_band_desc(desc);
	return desc.nom_out_tx_power - TxGain2TxPower(desc, tx_gains[chan]);
}

int soapy_device::getNominalTxPower(size_t chan)
{
	dev_band_desc desc;
	get_dev_band_desc(desc);

	return desc.nom_out_tx_power;
}

void soapy_device::log_ant_list(bool dir_tx, size_t chan, std::ostringstream& os)
{
}

//to del
int soapy_device::get_ant_idx(const std::string & name, bool dir_tx, size_t chan)
{
	return -1;
}

bool soapy_device::flush_recv(size_t num_pkts)
{

	return true;
}

bool soapy_device::setRxAntenna(const std::string & ant, size_t chan)
{
	device->setAntenna(SOAPY_SDR_RX, 0, ant.c_str()); // Set the RX antenna
	return true;
}

std::string soapy_device::getRxAntenna(size_t chan)
{
	return {};
}

bool soapy_device::setTxAntenna(const std::string & ant, size_t chan)
{
	device->setAntenna(SOAPY_SDR_TX, 0, ant.c_str()); // Set the TX antenna
	return true;
}

std::string soapy_device::getTxAntenna(size_t chan)
{
	return {};
}

bool soapy_device::requiresRadioAlign()
{
	return false;
}

GSM::Time soapy_device::minLatency() {
	/* UNUSED */
	return GSM::Time(0,0);
}


/// "timestamp_in" - This value is send to this method externally, is starts from 0
// and increased by "len" (SOAPY_PACKET_SIZE_SAMPLES) steps.
int soapy_device::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp_in, bool * underrun)
{
	int rc, expect_smpls;
	ssize_t avail_smpls;//Curenty in the rx buffer
	TIMESTAMP expect_timestamp;
	static uint64_t initial_samples = 0;//fixed once at the start, by received timestamp value

	static uint64_t prev_timestamp_ns = 0;
	//It is inctremented in this funcion
	static uint64_t internal_timestamp_samples = 0;

	static  int time_s_int_prev = 0;//test
	static bool startup_lock_flag = true;

	bool update_timestamp_flag = false;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*overrun = false;
	*underrun = false;


	/* Check that timestamp is valid */
	rc = rx_buffers[0]->avail_smpls(timestamp_in);
	if (rc < 0) {
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp_in);
		return 0;
	}

	/* Receive samples from HW until we have enough */
	while ((avail_smpls = rx_buffers[0]->avail_smpls(timestamp_in)) < len)
	{
		thread_enable_cancel(false);
		int flags;//flags set by receive operation
		long long timeNs; //Timestamp for receive buffer, nanosecnds
		void *buf_rx[] = {(short*)bufs[0]};
		expect_smpls = len - avail_smpls;

		int num_smpls = device->readStream(rxStream, buf_rx, expect_smpls, flags, timeNs, 50000); // 50ms timeout
		thread_enable_cancel(true);

		if (num_smpls <= 0)
		{
			LOGC(DDEV, ERROR) << "Device receive timed out (" << rc << " vs exp " << len << ").";
			return -1;
		}

		// In fact, we enter here when (timestamp_in+2500) block is filling with data
		if (!update_timestamp_flag)
		{
			update_timestamp_flag = true;//Protect of owerwrite in the next while()
			rx_timestamp_ns = timeNs; // For TX only
			rx_timestamp_in_samples = timestamp_in;
		}

		uint64_t diff_ns = timeNs - prev_timestamp_ns;
		prev_timestamp_ns = timeNs;
		//Difference between previous data read in samples
		uint64_t diff_samples = SoapySDR::timeNsToTicks(diff_ns, (double)SAMPLE_RATE_HZ);

		if ((diff_samples != SOAPY_PACKET_SIZE_SAMPLES) && startup_lock_flag)
		{
			LOGC(DDEV, ERROR) << "Received wrong start diff samples = " << diff_samples;
			continue;
		}
		startup_lock_flag = false;
		rx_is_stable = true;


		internal_timestamp_samples += diff_samples;

		//Do not use it directly, it give 2501 jumps sometimes (several time in 1s)
		//uint64_t rx_timestamp_samples = SoapySDR::timeNsToTicks(timeNs, (double)SAMPLE_RATE_HZ);
		uint64_t rx_timestamp_samples = internal_timestamp_samples;

		if (initial_samples == 0)
		{
			initial_samples = rx_timestamp_samples; // Fix value
		}

		rx_timestamp_samples = rx_timestamp_samples - initial_samples; // Remove start time offset

		if (expect_smpls != num_smpls)
		{
			LOGC(DDEV, NOTICE) << "Unexpected recv buffer len: expect "
							   << expect_smpls << " got " << num_smpls
							   << ", diff=" << expect_smpls - num_smpls;
		}

		expect_timestamp = timestamp_in + avail_smpls;
		if (expect_timestamp != (TIMESTAMP)rx_timestamp_samples)
		{
			LOGC(DDEV, ERROR) << "Unexpected recv buffer timestamp: expect "
							  << expect_timestamp << " got " << (TIMESTAMP)rx_timestamp_samples
							  << ", diff=" << (int32_t)(rx_timestamp_samples - expect_timestamp);
			LOGC(DDEV, NOTICE) << "avail_smpls: " << avail_smpls;
			LOGC(DDEV, NOTICE) << "timestamp_in: " << timestamp_in << " RX len: " << num_smpls;
		}

		//Put received data to the "rx_buffers"
		rc = rx_buffers[0]->write(bufs[0], num_smpls, (TIMESTAMP)rx_timestamp_samples);
		if (rc < 0)
		{
			LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc);
			LOGC(DDEV, ERROR) << rx_buffers[0]->str_status(timestamp_in);
			if (rc != smpl_buf::ERROR_OVERFLOW)
				return 0;
		}
		//LOGC(DDEV, NOTICE) << "put: " << num_smpls <<  " rx_timestamp_samples: " << rx_timestamp_samples;
	} //end of while

	//Copy all "len" data to "bufs"
	rc = rx_buffers[0]->read(bufs[0], len, timestamp_in);
	if ((rc < 0) || (rc != len))
	{
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc) << ". "
								<< rx_buffers[0]->str_status(timestamp_in)
								<< ", (len=" << len << ")";
		return 0;
	}

#ifdef SOAPY_LOOPBACK_TEST
	process_test_rx_data(timestamp_in, bufs[0]);
#endif

	//LOGC(DDEV, NOTICE) << "RX Timestamp  " << timeNs << " Samples " << rx_timestamp_samples;
	
    //usleep(2307);

	float time_s = (float)timestamp_in * 1.0f / (float)(SAMPLE_RATE_HZ);
    int time_s_int = (int)time_s;

    if (time_s_int_prev != time_s_int)
    {
        time_s_int_prev = time_s_int;
        LOGC(DDEV, NOTICE) << "RX TIME_S:" << time_s_int << std::endl;
    }

	return len;
}



int soapy_device::writeSamples(std::vector < short *>&bufs, int len,
			    bool * underrun, unsigned long long timestamp_in)
{
	static bool timestamp_lock = false; 

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*underrun = false;

    if (len != SOAPY_PACKET_SIZE_SAMPLES)
    {
		LOGC(DDEV, ERROR) << "WRONG LENGTH";
	}

	// Wait for RX to get first data packet
	while ((rx_timestamp_ns == 0) && (rx_is_stable == false))
	{
		usleep(500);
	}

	if ((rx_timestamp_in_samples >= timestamp_in) && (timestamp_lock == false))
	{
		//Simulate the we transmetted data to get a new bigger "timestamp_in"
		return len;
	}
	else
	{
		timestamp_lock = true;
	}

	//If we enter here, target TX "timestamp_in" is bigger (in future) that "rx_timestamp_in_samples"
	//TX Timestamp is in the future comparing to the received, so we need to increse value

	//Based on last RX data
	uint64_t realtime_tx_timestamp_ns = rx_timestamp_ns + 
		SoapySDR::ticksToTimeNs(SOAPY_PACKET_SIZE_SAMPLES * SOAPY_TX_OFFSET_PACKETS, (double)SAMPLE_RATE_HZ);
	uint64_t tx_timestamp_ns = realtime_tx_timestamp_ns;

	//Additional static offset
	tx_timestamp_ns -= SoapySDR::ticksToTimeNs(SOAPY_TX_OFFSET_SAMPLES, (double)SAMPLE_RATE_HZ);

#ifdef SOAPY_LOOPBACK_TEST
	generate_test_tx(timestamp_in);
	//Fill "test_tx_buf"
	void *buffs[] = {(short*)test_tx_buf};
#else
	void *buffs[] = {(short*)bufs[0]};
#endif

	thread_enable_cancel(false);

	int flags = SOAPY_SDR_HAS_TIME;

	// Write the buffer to the stream, this is blocking function, nearly 2314us.
	int ret = device->writeStream(
            txStream,         // The stream
            buffs,          // Array of buffer pointers
            SOAPY_PACKET_SIZE_SAMPLES,     // Number of samples
            flags,              // Flags (0 for no flags)
            (tx_timestamp_ns),              // Time in nanoSec (0 for immediate TX)
            100000          // Timeout in microseconds
        );

	//usleep(2314);
	thread_enable_cancel(true);

	if (ret < 0)
	{
		LOGC(DDEV, ERROR) << "Can't send data: " << ret;
		return 0;
	}

    return len;

}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

/// @brief Generate TX data ("test_tx_buf") that will be send to te radio
/// @param timestamp - timestamp in samples
void soapy_device::generate_test_tx(TIMESTAMP timestamp)
{
	//One step is 2.3ms
	uint32_t step = timestamp / SOAPY_PACKET_SIZE_SAMPLES; //0,1,2,3...

	uint8_t step_local = step % 64;
	//uint8_t pulses_num = (step_local < 3) ? (step_local + 1) : 0; //1,2,3 pulses

	if (step_local == 0)
		test_tx_timestamp = timestamp;//update test_tx_timestamp at the TX

	uint32_t pulse_length = SOAPY_PACKET_SIZE_SAMPLES / 6; //in samples
	uint32_t start_idx = SOAPY_PACKET_SIZE_SAMPLES / 2;//center of he packet
	uint32_t  stop_idx = start_idx + pulse_length;
	//i is a sample index
	for (uint32_t i = start_idx; i < stop_idx; i++)
	{
		float A = (step_local == 0) ? 5000.0f : 0.0f;
		uint32_t int_idx = i - start_idx;
		//uint32_t int_zone_cnt = int_idx * 3 / pulse_length;
		test_tx_buf[i*2] = (uint16_t)(A * cos(M_PI * int_idx / 10));
		test_tx_buf[i*2 + 1] = (uint16_t)(A * sin(M_PI * int_idx / 10));
	}
}

void soapy_device::process_test_rx_data(TIMESTAMP timestamp, int16_t* rx_data)
{
	uint32_t step = timestamp / SOAPY_PACKET_SIZE_SAMPLES; //0,1,2,3...

	uint16_t max_vale = 0;
	uint16_t start_rx_pos = 0;

	static uint64_t time_s_int_prev= 0;

	static uint16_t last_max_pos = 0;
	static uint64_t int_test_tx_step;
	static uint64_t int_test_rx_step;

	for (uint32_t i = 0; i < SOAPY_PACKET_SIZE_SAMPLES; i++)
	{
		int16_t curr_i_value = rx_data[i*2];
		if (curr_i_value > max_vale)
		{
			max_vale = curr_i_value;
		}

		if ((curr_i_value > 500) && (start_rx_pos == 0))
			start_rx_pos = i; //Fix received pulse start position
	}

	// Signal was detected
	if (max_vale > 500)
	{
		last_max_pos = start_rx_pos;
		int_test_tx_step = test_tx_timestamp / SOAPY_PACKET_SIZE_SAMPLES;
		int_test_rx_step = step;
	}

	float time_s = (float)timestamp * 1.0f / (float)(SAMPLE_RATE_HZ);
    uint64_t time_s_int = (uint64_t)time_s;


    if (time_s_int_prev != time_s_int)
    {
        time_s_int_prev = time_s_int;
        LOGC(DDEV, NOTICE) << "MAX RX POS:" << last_max_pos << std::endl;
		LOGC(DDEV, NOTICE) << "TX step:" << int_test_tx_step << " RX step:" << int_test_rx_step << std::endl;
    }

	if ((step < 2000) && (max_vale > 500))
	{
		LOGC(DDEV, NOTICE) << "RX MAX: " << max_vale << " step: " << step << " pos: " << start_rx_pos;
	}
}

void soapy_device::tx_debug_delay(uint32_t time_us)
{
    auto t_now = std::chrono::steady_clock::now();
    auto start_us = std::chrono::duration_cast<std::chrono::microseconds>(t_now.time_since_epoch()).count();
    uint64_t delay_us = 0;
    while (delay_us < time_us)
    {
        auto t_now2 = std::chrono::steady_clock::now();
        auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(t_now2.time_since_epoch()).count();
        delay_us = now_us - start_us;
    }
}

bool soapy_device::updateAlignment(TIMESTAMP timestamp)
{
	return true;
}

bool soapy_device::setTxFreq(double wFreq, size_t chan)
{
	uint16_t req_arfcn;
	enum gsm_band req_band;

	if (chan >= chans) {
		LOGC(DDEV, ALERT) << "Requested non-existent channel " << chan;
		return false;
	}

	LOGCHAN(chan, DDEV, NOTICE) << "Setting Tx Freq to " << wFreq << " Hz";

	req_arfcn = gsm_freq102arfcn(wFreq / 1000 / 100 , 0);
	if (req_arfcn == 0xffff) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown ARFCN for Tx Frequency " << wFreq / 1000 << " kHz";
		return false;
	}
	if (gsm_arfcn2band_rc(req_arfcn, &req_band) < 0) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown GSM band for Tx Frequency " << wFreq
					   << " Hz (ARFCN " << req_arfcn << " )";
		return false;
	}

	if (!set_band(req_band))
		return false;

	uint64_t target_freq_hz = (uint64_t)wFreq;
	device->setFrequency(SOAPY_SDR_TX, 0, (double)target_freq_hz);

	return true;
}

bool soapy_device::setRxFreq(double wFreq, size_t chan)
{
	uint16_t req_arfcn;
	enum gsm_band req_band;

	req_arfcn = gsm_freq102arfcn(wFreq / 1000 / 100, 1);
	if (req_arfcn == 0xffff) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown ARFCN for Rx Frequency " << wFreq / 1000 << " kHz";
		return false;
	}
	if (gsm_arfcn2band_rc(req_arfcn, &req_band) < 0) {
		LOGCHAN(chan, DDEV, ALERT) << "Unknown GSM band for Rx Frequency " << wFreq
					   << " Hz (ARFCN " << req_arfcn << " )";
		return false;
	}

	if (!set_band(req_band))
		return false;

#ifdef SOAPY_LOOPBACK_TEST
	wFreq += 45e6;//same as TX
#endif

	LOGCHAN(chan, DDEV, NOTICE) << "Setting Rx Freq to " << wFreq << " Hz";

	uint64_t target_freq_hz = (uint64_t)wFreq;
	device->setFrequency(SOAPY_SDR_RX, 0, (double)target_freq_hz);

	return true;
}


void soapy_device::init_gains()
{
    //int res;

    int tx_gain_db = SOAPY_TX_GAIN_DB;
	int rx_gain_db = SOAPY_RX_GAIN_DB;

    device->setGain(SOAPY_SDR_TX, 0, (double)tx_gain_db);
	device->setGain(SOAPY_SDR_RX, 0, (double)rx_gain_db);

	LOGC(DDEV, NOTICE) << "TX gain set to " << tx_gain_db << " dB";
	LOGC(DDEV, NOTICE) << "RX gain set to " << rx_gain_db << " dB";
}

void soapy_device::set_rates_tx()
{
 	//int res;
	// Set the sample rate
	device->setSampleRate(SOAPY_SDR_TX, 0, SAMPLE_RATE_HZ);
	device->setBandwidth(SOAPY_SDR_TX, 0, 2e6);//2MHz

 	//tx_rate = rx_rate = samp_rate_hz;
	LOGC(DDEV, NOTICE) << "TX samplerate was set to " << SAMPLE_RATE_HZ << " Hz";
}

void soapy_device::set_rates_rx()
{
 	//int res;
	// Set the sample rate
	device->setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE_HZ);
	device->setBandwidth(SOAPY_SDR_RX, 0, 1.5e6);//2MHz

	double real_freq_hz = device->getSampleRate(SOAPY_SDR_RX, 0);

 	//tx_rate = rx_rate = samp_rate_hz;
	LOGC(DDEV, NOTICE) << "RX target samplerate " << SAMPLE_RATE_HZ << " Hz";

	LOGC(DDEV, NOTICE) << "RX samplerate was set to " << real_freq_hz << " Hz";
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "SDR Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "SDR doesn't support lo_offset";
		return NULL;
	}
	return new soapy_device(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}