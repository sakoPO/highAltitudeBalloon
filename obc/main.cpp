/**
 * @license    BSD 3-Clause
 * @copyright  microHAL
 * @version    $Id$
 * @brief      diagnostic example main file
 *
 * @authors    Pawel Okas
 * created on: 2016
 * last modification: <DD-MM-YYYY>
 *
 * @copyright Copyright (c) 2016, Pawel Okas
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *     1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
 *        software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <cstring>
#include <thread>

#include "diagnostic/diagnostic.h"
#include "microhal.h"
#include "obc_bsp.h"
#include "sd.h"
#include "lepton.h"
#include "uCamII.h"
#include "os/microhal_semaphore.h"

#include "CRCLib/crc32.h"

using namespace microhal;
using namespace microhal::diagnostic;
using namespace std::literals::chrono_literals;

struct Metadata {
	uint16_t dataSize;
	float latitude;
	float longitude;
	float altitude;
	uint32_t metadataCrc;
	uint32_t leptonCrc;
	uint32_t vgaCrc;
	uint32_t allCrc;

	static constexpr size_t serializedSize = sizeof(dataSize) + sizeof(latitude) + sizeof(longitude) + sizeof(altitude) + sizeof(metadataCrc)
			+ sizeof(leptonCrc) + sizeof(vgaCrc) + sizeof(allCrc);
} MICROHAL_PACKED;


static const uint8_t header[] = "HighAltitudeBalloonHEADER_HighAltitudeBalloonHEADER";
static constexpr size_t headerSize = sizeof(header);

struct Pictures {
	uint8_t header[headerSize];
	Metadata metadata;
	uint8_t leptonPicture[60][80];
	uint8_t vgaPicture[40000];
	size_t vgaSize;
} MICROHAL_PACKED;

struct Buffer {
	Pictures data;
	uint8_t padding[512 - (sizeof(Pictures) % 512)];
} MICROHAL_PACKED;

static_assert(sizeof(Buffer) % 512 == 0, "Error, Buffer size should be multiple of 512");

static_assert(sizeof(Metadata) == Metadata::serializedSize, "");

uint16_t writePicturesOnSDCard(Sd &sdCard, uint32_t startPageNumber, const Pictures &pictures) {//const uint8_t leptonPicture[60*80], const uint8_t *vgaData, size_t vgaSize) {
	const void *dataPtr = &pictures;

	size_t dataSize = pictures.metadata.dataSize + headerSize;
	uint32_t blockCount = dataSize /512;
	if (dataSize % 512) {
		blockCount++;
	}

	sdCard.writeMultipleBlock(dataPtr, startPageNumber, blockCount);
	return blockCount;
}

microhal::os::Semaphore sem;


Buffer buffer_;
Buffer buffer_2;
Pictures &BufferA = buffer_.data;
Pictures &BufferB = buffer_2.data;
Pictures *buffer = &BufferA;

void SDcartWritingTask() {
	static Sd sdCard(sdCardSPI, sdCardCs);
	diagChannel << lock << MICROHAL_INFORMATIONAL << "Starting SD Card thread" << unlock;
	while(1) {
		if (sdCard.init()) {
			diagChannel << lock << MICROHAL_INFORMATIONAL << "SD Card initialized successfully." << unlock;
		} else {
			diagChannel << lock << MICROHAL_CRITICAL << "Unable to initialize SD Card, trying again..." << unlock;
			continue;
		}
		stm32f4xx::SPI::spi3.setPrescaler(stm32f4xx::SPI::PRESCALER_2);
		uint32_t blockNumber = 0;

		while(1) {
			if (sem.wait(std::chrono::milliseconds::max())) {
				blockNumber += writePicturesOnSDCard(sdCard, blockNumber, *buffer);
				diagChannel << lock << MICROHAL_INFORMATIONAL << "Lepton picture added to SD Card." << endl << unlock;
			}
		}
	}
}

bool initVGACam(uCAM_II &uCam) {
//    cameraPort.setBaudRate(115200);
//    cameraPort.setDataBits(SerialPort::DataBits::Data8);
//    cameraPort.setStopBits(SerialPort::StopBits::OneStop);
//    cameraPort.setParity(SerialPort::Parity::NoParity);

    if (!uCam.sync()) {
        diagChannel << lock << MICROHAL_ERROR << "Unable to synchronize with camera." << unlock;
        return false;
    }

//    if (!uCam.setBaudrate(uCAM_II::Baudrate::Baud_921600)) {
//    	diagChannel << lock << MICROHAL_ERROR << "Unable to set uCam baudrate." << unlock;
//    }
//    cameraPort.setBaudRate(921600);

    if (!uCam.initJPEG(uCAM_II::JpegResolution::Image_640x480)) {
        diagChannel << lock << MICROHAL_ERROR << "Unable to initialize camera." << unlock;
    }

    if (!uCam.setPackageSize(512)) {
        diagChannel << lock << MICROHAL_ERROR << "Unable to set package size" << unlock;
    }
    return true;
}

void updateMetadata(Metadata &metadata, const Pictures& buffer) {
	metadata.dataSize = sizeof(Metadata) + 60*80 + buffer.vgaSize;
	metadata.latitude = 1;
	metadata.longitude = 1;
	metadata.altitude = 100;
	metadata.metadataCrc = microhal::crc::crc32(&metadata, sizeof(metadata.dataSize) + sizeof(metadata.latitude) + sizeof(metadata.longitude)
								  	  	  	  	  	  	  	  + sizeof(metadata.altitude));
	metadata.leptonCrc = microhal::crc::crc32(buffer.leptonPicture, 60*80);
	metadata.vgaCrc = microhal::crc::crc32(buffer.vgaPicture, buffer.vgaSize);
}


int main(void) {
	std::this_thread::sleep_for(1s);

    debugPort.write("\n\r------------------- Balloon OBC -------------------------\n\r");
//    diagChannel.setOutputDevice(debugPort);
    // lets check if diagChannal is working
    diagChannel << lock << MICROHAL_EMERGENCY << "Information from diagnostic channel." << unlock;

    std::thread sdCard1(SDcartWritingTask);
    vTaskPrioritySet(sdCard1.native_handle(), uxTaskPriorityGet(NULL));

    static Lepton lepton(leptonSPI, leptonI2C, leptonCS, leptonPower, leptonReset);
    static uCAM_II uCam(cameraPort);

    initVGACam(uCam);
    lepton.startup();
    Pictures *activeBuffer = &BufferA;
    while (1) {
    	auto beginTime = std::chrono::system_clock::now();
        // --------------------------------------- get snapshot JPEG picture --------------------------------------------------------
//    	if (!uCam.snapshot(uCAM_II::SnapshotType::CompressedPictureJPEG)) {
//    		diagChannel << lock << MICROHAL_INFORMATIONAL << "Unable to take snapshot" << unlock;
//    	}
    	while (1) {
			if (lepton.isNewPictureAvailable()) {
				for (size_t y = 0; y < 60; y++) {
					for (size_t x = 0; x < 80; x++) {
						size_t i = y*80 + x;
						activeBuffer->leptonPicture[y][x] = lepton.getPicture()[i*2 + 1];
					}
				}
				diagChannel << lock << MICROHAL_INFORMATIONAL << "Lepton picture received, adding to data queue." << unlock;
				break;
			}

			if (beginTime + 150ms < std::chrono::system_clock::now()) {
				diagChannel << lock << MICROHAL_INFORMATIONAL << "Timeout while receiving Lepton picture." << unlock;
				break;
			}

			lepton.timeProc();
			std::this_thread::sleep_for(1ms);
		}

		size_t tmpSize = 0;
		if (!uCam.getPicture(uCAM_II::PictureType::JPEGPictureMode, buffer->vgaPicture, tmpSize)) {
			diagChannel << lock << MICROHAL_INFORMATIONAL << "Unable to read picture. Sending reset" << unlock;
			uCam.reset(uCAM_II::ResetType::StateMachineReset);
			while(1);
		} else {
			diagChannel << lock << MICROHAL_INFORMATIONAL << "uCam picture received, adding to data queue." << unlock;
		}
		activeBuffer->vgaSize = tmpSize;
		memcpy(activeBuffer->header, header, headerSize);
		updateMetadata(activeBuffer->metadata, *activeBuffer);

		buffer = activeBuffer;
		if (activeBuffer == &BufferA) {
			activeBuffer = &BufferB;
		} else {
			activeBuffer = &BufferA;
		}
		sem.give();

    	std::this_thread::sleep_until(beginTime + 500ms);
	}

    return 0;
}
