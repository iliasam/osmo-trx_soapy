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


#define SAMPLE_BUF_SZ    (1 << 20) /* Size of Rx timestamp based Ring buffer, in bytes */


#define PUT_PACKET_SIZE_SAMPLES     2500

#define BUFFER_SIZE_BYTES           PUT_PACKET_SIZE_SAMPLES * 3

#define SAMPLE_RATE_HZ              (GSMRATE * tx_sps)


#define DEV_HW_TYPE                 NORMAL
//#define DEV_HW_TYPE                 RESAMP_SOAPY1
	//return RESAMP_SOAPY1; //<<<<<<< Using RadioInterfaceResamp
	//return RESAMP_64M;

/* Device parameter descriptor */
struct dev_desc {
	/* Does LimeSuite allow switching the clock source for this device?
	 * LimeSDR-Mini does not have switches but needs soldering to select
	 * external/internal clock. Any call to LMS_SetClockFreq() will fail.
	 */
	bool clock_src_switchable;
	/* Does LimeSuite allow using REF_INTERNAL for this device?
	 * LimeNET-Micro does not like selecting internal clock
	 */
	bool clock_src_int_usable;
	/* Sample rate coef (without having TX/RX samples per symbol into account) */
	double rate;
	/* Sample rate coef (without having TX/RX samples per symbol into account), if multi-arfcn is enabled */
	double rate_multiarfcn;
	/* Coefficient multiplied by TX sample rate in order to shift Tx time */
	double ts_offset_coef;
	/* Coefficient multiplied by TX sample rate in order to shift Tx time, if multi-arfcn is enabled */
	double ts_offset_coef_multiarfcn;
	/* Device Name Prefix as presented by LimeSuite API LMS_GetDeviceInfo() */
	std::string name_prefix;
};

static const std::map<enum soapy_dev_type, struct dev_desc> dev_param_map {
	{ SOAPY_TYPE1,   { true,  true,  GSMRATE, MCBTS_SPACING, 8.9e-5, 7.9e-5, "SOAPY DEV" } },
};

typedef std::tuple<soapy_dev_type, enum gsm_band> dev_band_key;
typedef std::map<dev_band_key, dev_band_desc>::const_iterator dev_band_map_it;
static const std::map<dev_band_key, dev_band_desc> dev_band_nom_power_param_map {
	{ std::make_tuple(SOAPY_TYPE1, GSM_BAND_900),	{ 73.0, 10.8,  -6.0  } },
};

/* So far measurements done for B210 show really close to linear relationship
 * between gain and real output power, so we simply adjust the measured offset
 */
static double TxGain2TxPower(const dev_band_desc &desc, double tx_gain_db)
{
	return desc.nom_out_tx_power - (desc.nom_lms_tx_gain - tx_gain_db);
}
static double TxPower2TxGain(const dev_band_desc &desc, double tx_power_dbm)
{
	return desc.nom_lms_tx_gain - (desc.nom_out_tx_power - tx_power_dbm);
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

	log_file.open("record_u24.sdriq", std::ofstream::out | std::ofstream::binary);
	if (log_file.is_open())
  	{
    	LOGC(DDEV, NOTICE) << "LOG IS OPEN";
  	}

	rx_buffers.resize(chans);

	/* Set up per-channel Rx timestamp based Ring buffers */
	rx_buffers[0] = new smpl_buf(SAMPLE_BUF_SZ / sizeof(uint32_t));

}

soapy_device::~soapy_device()
{
	unsigned int i;
	LOGC(DDEV, NOTICE) << "Closing SOAPY device";

    if (device) {
		device->deactivateStream(txStream);
        device->closeStream(txStream);
		device->deactivateStream(rxStream);
        device->closeStream(rxStream);
		SoapySDR::Device::unmake(device);
	}

	if (log_file.is_open())
  	{
    	log_file.close();
  	}

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
	tx_running = false;

	m_dev_type = soapy_dev_type::SOAPY_TYPE1;

	SoapySDR::KwargsList sdr_results = SoapySDR::Device::enumerate("driver=plutosdr");

	if (sdr_results.empty()) 
	{
    	LOGC(DDEV, ERROR) << "Soapy device was not found!";
		return -1;
	}

	//device = SoapySDR::Device::make("driver=plutosdr,usb_direct=1,timestamp_every=2500,loopback=0");
	device = SoapySDR::Device::make("driver=plutosdr,direct=1,loopback=0,timestamp_every=2500");

	if (device == nullptr) {
    	LOGC(DDEV, ERROR) << "Soapy device opening error!";
		return -1;
	}	

	ts_initial = 0;//do not delete!

	rx_timestamp_ns = 0;
	rx_timestamp_samples = 0;

	set_rates_tx();
	set_rates_rx();
	init_gains();

	/*
	SoapySDR::Kwargs deviceArgs;
	deviceArgs["bufflen"] = "2500";
	txStream = device->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, std::vector<size_t>(), deviceArgs);
	rxStream = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, std::vector<size_t>(), deviceArgs);
	*/

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

	//test_rx();

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
	uint16_t tmp_rx_buf[PUT_PACKET_SIZE_SAMPLES * 2];//iq
	void *buf_rx[] = {tmp_rx_buf};

	device->setFrequency(SOAPY_SDR_RX, 0, (double)900e6);

	uint64_t diff_ns_prev = 0;
	uint64_t diff_sps_prev = 0;

	for (int i = 0; i < 400; i++)
	{
		thread_enable_cancel(false);
		int num_smpls = device->readStream(rxStream, buf_rx, PUT_PACKET_SIZE_SAMPLES, flags, timeNs, 50000); // 50ms timeout
		uint64_t rx_timestamp_samples = SoapySDR::timeNsToTicks(timeNs, (double)SAMPLE_RATE_HZ);

		uint64_t diff_ns = timeNs - diff_ns_prev;
		diff_ns_prev = timeNs;

		uint64_t diff_sps = rx_timestamp_samples - diff_sps_prev;
		diff_sps_prev = rx_timestamp_samples;

		uint64_t rx_diff_samples = SoapySDR::timeNsToTicks(diff_ns, (double)SAMPLE_RATE_HZ);

		if (diff_sps != PUT_PACKET_SIZE_SAMPLES)
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

	//if (!restart())
	//	return false;

	started = true;
	return true;
}

bool soapy_device::stop()
{
	//unsigned int i;

	LOGC(DDEV, NOTICE) << "dev stop";

	if (!started)
		return true;

	tx_running = false;

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

	LOGCHAN(chan, DDEV, NOTICE) << "Setting RX gain to " << dB << " dB";

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

	//if (LMS_SetGaindB(m_lms_dev, LMS_CH_TX, chan, dB) < 0)
	//	LOGCHAN(chan, DDEV, ERR) << "Error setting TX gain to " << dB << " dB (~" << tx_power << " dBm)";
	//else
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
	return true;
}

std::string soapy_device::getRxAntenna(size_t chan)
{
	return {};
}

bool soapy_device::setTxAntenna(const std::string & ant, size_t chan)
{
	//ilia todo
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
	/* UNUSED on limesdr (only used on usrp1/2) */
	return GSM::Time(0,0);
}
/// "timestamp_in" - This value is send to this method externally, is starts from 0
// and increased by "len" (PUT_PACKET_SIZE_SAMPLES) steps.
int soapy_device::readSamples(std::vector < short *>&bufs, int len, bool * overrun,
			   TIMESTAMP timestamp_in, bool * underrun)
{
	int rc, expect_smpls;
	ssize_t avail_smpls;//Curenty in the rx buffer
	TIMESTAMP expect_timestamp;
	unsigned int i;
	//static int bad_cnt = 0;
	//static int good_cnt = 0;
	static uint64_t initial_samples = 0;//fixed once at the start, by received timestamp value

	static uint64_t prev_timestamp_ns = 0;
	//It is inctremented in this funcion
	static uint64_t internal_timestamp_samples = 0;

	static  int time_s_int_prev = 0;//test

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
		long long timeNs; //timestamp for receive buffer, nanosecnds
		void *buf_rx[] = {(short*)bufs[0]};
		expect_smpls = len - avail_smpls;

		int num_smpls =  device->readStream(rxStream, buf_rx, expect_smpls, flags, timeNs, 50000); // 50ms timeout
		thread_enable_cancel(true);

		if (num_smpls <= 0)
		{
			LOGC(DDEV, ERROR) << "Device receive timed out (" << rc << " vs exp " << len << ").";
			return -1;
		}

		// In fact, we enter here then (timestamp_in+2500) block is filling with data
		if (!update_timestamp_flag)
		{
			update_timestamp_flag = true;//Protect of owerwrite in the next while()
			rx_timestamp_ns = timeNs;
			rx_timestamp_samples = timestamp_in;
		}

		uint64_t diff_ns = timeNs - prev_timestamp_ns;
		prev_timestamp_ns = timeNs;
		//Difference before previous data read in samples
		uint64_t diff_samples = SoapySDR::timeNsToTicks(diff_ns, (double)SAMPLE_RATE_HZ);

		internal_timestamp_samples += diff_samples;

		//Do not use it directly, it give 2501 jumps sometimes (several time in 1s)
		//uint64_t rx_timestamp_samples = SoapySDR::timeNsToTicks(timeNs, (double)SAMPLE_RATE_HZ);
		uint64_t rx_timestamp_samples = internal_timestamp_samples;

		if (initial_samples == 0)
			initial_samples = rx_timestamp_samples;					   // Fix value
		rx_timestamp_samples = rx_timestamp_samples - initial_samples; // Remove start time offset

		LOGC(DDEV, DEBUG) << "Received timestamp = " << (TIMESTAMP)rx_timestamp_samples << " (" << num_smpls << ")";

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
	}

	//Copy all "len" data to "bufs"
	rc = rx_buffers[0]->read(bufs[0], len, timestamp_in);
	if ((rc < 0) || (rc != len))
	{
		LOGC(DDEV, ERROR) << rx_buffers[0]->str_code(rc) << ". "
								<< rx_buffers[0]->str_status(timestamp_in)
								<< ", (len=" << len << ")";
		return 0;
	}

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
			    bool * underrun, unsigned long long timestamp)
{
	int rc = 0;
	static int time_s_int_prev = 0;
	static uint64_t start_time_s = 0;

	static int tx_cnt = 0;

	static bool timestamp_lock0 = false; 
	static bool timestamp_lock = false; 
	static uint64_t prev_timestamp_ns = 0;

	if (bufs.size() != chans) {
		LOGC(DDEV, ERROR) << "Invalid channel combination " << bufs.size();
		return -1;
	}

	*underrun = false;

    if (len != PUT_PACKET_SIZE_SAMPLES)
    {
        LOGC(DDEV, ERROR) << "WRONG LENGTH";
    }

	// Wait for RX to get first data packet
	while (rx_timestamp_ns == 0)
	{
		usleep(500);
	}

	if ((rx_timestamp_samples >= timestamp) && (timestamp_lock0 == false))
	{
		//Simulate the we transmetted data to get a new bigger "timestamp"
		return len;
	}
	else
	{
		timestamp_lock0 = true;
	}

	//If we enter here, target TX "timestamp" is bigger (in future) that "rx_timestamp_samples"
	//TX Timestamp is in the future comparing to the received, so we need to increse value

	uint64_t tx_timestamp_ns = 0;
	if (!timestamp_lock)
	{
		timestamp_lock = true;
		tx_timestamp_ns = rx_timestamp_ns + SoapySDR::ticksToTimeNs(PUT_PACKET_SIZE_SAMPLES * 2, (double)SAMPLE_RATE_HZ);
	}
	else
	{
		//Just add fixed value
		 tx_timestamp_ns = prev_timestamp_ns + SoapySDR::ticksToTimeNs(PUT_PACKET_SIZE_SAMPLES, (double)SAMPLE_RATE_HZ);
	}
	prev_timestamp_ns = tx_timestamp_ns;

	void *buffs[] = {(short*)bufs[0]};

	thread_enable_cancel(false);

	int flags = SOAPY_SDR_HAS_TIME;

	auto t_now = std::chrono::steady_clock::now();
    auto start_us = std::chrono::duration_cast<std::chrono::microseconds>(t_now.time_since_epoch()).count();

	// Write the buffer to the stream, this is blocking function, nearly 2314us.
	int ret = device->writeStream(
            txStream,         // The stream
            buffs,          // Array of buffer pointers
            PUT_PACKET_SIZE_SAMPLES,     // Number of samples
            flags,              // Flags (0 for no flags)
            (tx_timestamp_ns + 5),              // Time in nanoSec (0 for immediate TX)
            100000          // Timeout in microseconds
        );

	//int ret = 0;
	//usleep(2314);
	thread_enable_cancel(true);

	auto t_now2 = std::chrono::steady_clock::now();
    auto stop_us = std::chrono::duration_cast<std::chrono::microseconds>(t_now2.time_since_epoch()).count();

	tx_cnt++;

	if (ret < 0)
	{
		LOGC(DDEV, ERROR) << "Can't send data: " << ret;
		return 0;
	}

	uint64_t delay_us = stop_us - start_us;

	if (tx_cnt < 20)
	{
		//LOGC(DDEV, NOTICE) << "Duration us:" << delay_us << std::endl;
		//LOGC(DDEV, NOTICE) << "RET:" << ret << std::endl;
		//LOGC(DDEV, NOTICE) << "TIME US:" << stop_us << std::endl;
		//LOGC(DDEV, NOTICE) << "*" << std::endl;
	}
	
    float time_s = (float)timestamp * 1.0f / (float)(SAMPLE_RATE_HZ);
    int time_s_int = (int)time_s;

    if (time_s_int_prev != time_s_int)
    {
        time_s_int_prev = time_s_int;
        //LOGC(DDEV, NOTICE) << "TX TIME_S:" << time_s_int << std::endl;
        //LOGC(DDEV, NOTICE) << "SAMPLES:" << len << std::endl;

		LOGC(DDEV, NOTICE) << "Duration us:" << delay_us << std::endl;
    }
    return len;

}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

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

	LOGCHAN(chan, DDEV, NOTICE) << "Setting Rx Freq to " << wFreq << " Hz";

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

	uint64_t target_freq_hz = (uint64_t)wFreq;
	device->setFrequency(SOAPY_SDR_RX, 0, (double)target_freq_hz);

	return true;
}


void soapy_device::init_gains()
{
    //int res;

    int tx_gain_db = 60;
	int rx_gain_db = 40;

    device->setGain(SOAPY_SDR_TX, 0, (double)tx_gain_db);
	device->setGain(SOAPY_SDR_RX, 0, (double)rx_gain_db);

	device->setAntenna(SOAPY_SDR_TX, 0, "A"); // Set the TX antenna
	device->setAntenna(SOAPY_SDR_RX, 0, "A_BALANCED"); // Set the RX antenna

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
	ts_offset = 60; // FIXME: actual blade offset, should equal b2xx
}

void soapy_device::set_rates_rx()
{
 	//int res;
	// Set the sample rate
	device->setSampleRate(SOAPY_SDR_RX, 0, SAMPLE_RATE_HZ);
	device->setBandwidth(SOAPY_SDR_RX, 0, 2e6);//2MHz

	double real_freq_hz = device->getSampleRate(SOAPY_SDR_RX, 0);

 	//tx_rate = rx_rate = samp_rate_hz;
	LOGC(DDEV, NOTICE) << "RX target samplerate " << SAMPLE_RATE_HZ << " Hz";

	LOGC(DDEV, NOTICE) << "RX samplerate was set to " << real_freq_hz << " Hz";
	ts_offset = 60; // FIXME: actual blade offset, should equal b2xx
}

RadioDevice *RadioDevice::make(size_t tx_sps, size_t rx_sps,
			       InterfaceType iface, size_t chans, double lo_offset,
			       const std::vector < std::string > &tx_paths,
			       const std::vector < std::string > &rx_paths)
{
	if (tx_sps != rx_sps) {
		LOGC(DDEV, ERROR) << "LMS Requires tx_sps == rx_sps";
		return NULL;
	}
	if (lo_offset != 0.0) {
		LOGC(DDEV, ERROR) << "LMS doesn't support lo_offset";
		return NULL;
	}
	return new soapy_device(tx_sps, rx_sps, iface, chans, lo_offset, tx_paths, rx_paths);
}