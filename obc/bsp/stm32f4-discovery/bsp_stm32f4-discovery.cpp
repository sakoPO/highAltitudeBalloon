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

#include "SPIDevice/SPIDevice.h"
#include "microhal.h"
#include "obc_bsp.h"

using namespace microhal;
using namespace stm32f4xx;

extern "C" int main(int, void *);

static void run_main(void *) {
    main(0, nullptr);
}

#define 	NVIC_PriorityGroup_4   ((uint32_t)0x300)
#define AIRCR_VECTKEY_MASK ((uint32_t)0x05FA0000)


/* The prototype shows it is a naked function - in effect this is just an
assembly function. */
extern "C"  void HardFault_Handler( void ) __attribute__( ( naked ) );

/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
extern "C"  void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void printErrorMsg(const char * errMsg)
{
   while(*errMsg != '0'){
      ITM_SendChar(*errMsg);
      ++errMsg;
   }
}


extern "C" void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
/* These are volatile to try and prevent the compiler/linker optimising them
away as the variables never actually get used.  If the debugger won't show the
values of the variables, make them global my moving their declaration outside
of this function. */
volatile uint32_t r0;
volatile uint32_t r1;
volatile uint32_t r2;
volatile uint32_t r3;
volatile uint32_t r12;
volatile uint32_t lr; /* Link register. */
volatile uint32_t pc; /* Program counter. */
volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[ 0 ];
    r1 = pulFaultStackAddress[ 1 ];
    r2 = pulFaultStackAddress[ 2 ];
    r3 = pulFaultStackAddress[ 3 ];

    r12 = pulFaultStackAddress[ 4 ];
    lr = pulFaultStackAddress[ 5 ];
    pc = pulFaultStackAddress[ 6 ];
    psr = pulFaultStackAddress[ 7 ];

    /* When the following line is hit, the variables contain the register values. */

    static char msg[250];
    printErrorMsg("In Hard Fault Handler\n");
    uint8_t i = sprintf(msg, "In Hard Fault Handler\n");
    i += sprintf(&msg[i], "SCB->HFSR = 0x%08x\n", SCB->HFSR);
    printErrorMsg(msg);
    if ((SCB->HFSR & (1 << 30)) != 0) {
    	printErrorMsg("Forced Hard Fault\n");
    	i += sprintf(&msg[i], "Forced Hard Fault\n");
    	auto cfsr = SCB->CFSR;
    	uint8_t busFaultStatusRegister = (cfsr & 0xFF00) >> 8;
    	enum {
    		BFARVALID =   0b10000000,
			LSPERR =      0b00100000,
			STKERR =      0b00010000,
			UNSTKERR =    0b00001000,
			IMPRECISERR = 0b00000100,
			PRECISERR =   0b00000010,
			IBUSERR =     0b00000001
    	};
    	if (busFaultStatusRegister) {
    		i += sprintf(&msg[i], "Bus Fault detected\n");
    		if (busFaultStatusRegister & BFARVALID) {
    			// register BFAR holds fault address
				i += sprintf(&msg[i], "Fault cause address, SCB-BFAR: 0x%08x\n", SCB->BFAR);
    		}
    		if (busFaultStatusRegister & LSPERR) {
    			i += sprintf(&msg[i], "a bus fault occurred during floating-point lazy state preservation\n");
    		}
    		if (busFaultStatusRegister & STKERR) {
    			i += sprintf(&msg[i], "stacking for an exception entry has caused one or more BusFaults\n");
    		}
    		if (busFaultStatusRegister & UNSTKERR) {
    			i += sprintf(&msg[i], "unstack for an exception return has caused one or more BusFaults\n");
    		}
    		if (busFaultStatusRegister & IMPRECISERR) {
    			i += sprintf(&msg[i], "Imprecise data bus error\n");
    		}
    		if (busFaultStatusRegister & PRECISERR) {
    			i += sprintf(&msg[i], "Precise data bus error\n");
    		}
    		if (busFaultStatusRegister & IBUSERR) {
    			i += sprintf(&msg[i], "Instruction bus error\n");
    		}

    	}

    	i += sprintf(&msg[i], "SCB->CFSR = 0x%08x\n", SCB->CFSR );


    	printErrorMsg(msg);
    }

    asm volatile("BKPT #01");
    for( ;; );
}

extern "C" void * __real_memcpy( void * destination, const void * source, size_t num );
extern "C" void *__wrap_memcpy( void * destination, const void * source, size_t num ) {
	void *lastRAMaddr;
	lastRAMaddr = (void*)(0x20000000 + (112*1024));

	if (destination > lastRAMaddr || destination + num > lastRAMaddr) {
		asm volatile("BKPT #01");
	}

	return __real_memcpy(destination, source, num);
}



void hardwareConfig(void) {
    Core::pll_start(8000000, 168000000);
    Core::fpu_enable();

    IOManager::routeSerial<3, Txd, stm32f4xx::GPIO::PortD, 8>();
    IOManager::routeSerial<3, Rxd, stm32f4xx::GPIO::PortD, 9>();

    IOManager::routeSerial<2, Txd, stm32f4xx::GPIO::PortA, 2>();
    IOManager::routeSerial<2, Rxd, stm32f4xx::GPIO::PortA, 3>();

    IOManager::routeSPI<1, SCK, stm32f4xx::GPIO::PortA, 5>();
    IOManager::routeSPI<1, MISO, stm32f4xx::GPIO::PortA, 6>();
    IOManager::routeSPI<1, MOSI, stm32f4xx::GPIO::PortA, 7>();

    IOManager::routeSPI<3, SCK, stm32f4xx::GPIO::PortC, 10>();
    IOManager::routeSPI<3, MISO, stm32f4xx::GPIO::PortB, 4>(stm32f4xx::GPIO::PullType::PullUp);
    IOManager::routeSPI<3, MOSI, stm32f4xx::GPIO::PortB, 5>();

    IOManager::routeI2C<2, SDA, stm32f4xx::GPIO::PortB, 11>();
    IOManager::routeI2C<2, SCL, stm32f4xx::GPIO::PortB, 10>();

    debugPort.setDataBits(stm32f4xx::SerialPort::Data8);
    debugPort.setStopBits(stm32f4xx::SerialPort::OneStop);
    debugPort.setParity(stm32f4xx::SerialPort::NoParity);
    debugPort.open(stm32f4xx::SerialPort::ReadWrite);
    debugPort.setBaudRate(stm32f4xx::SerialPort::Baud115200);
    microhal::diagnostic::diagChannel.setOutputDevice(debugPort);

    cameraPort.setDataBits(stm32f4xx::SerialPort::Data8);
    cameraPort.setStopBits(stm32f4xx::SerialPort::OneStop);
    cameraPort.setParity(stm32f4xx::SerialPort::NoParity);
    cameraPort.open(stm32f4xx::SerialPort::ReadWrite);
    //cameraPort.setBaudRate(stm32f4xx::SerialPort::Baud115200);
    cameraPort.setBaudRate(921600);

    stm32f4xx::SPI::spi1.init(stm32f4xx::SPI::Mode3, stm32f4xx::SPI::PRESCALER_16);
    stm32f4xx::SPI::spi1.enable();

    stm32f4xx::SPI::spi3.init(stm32f4xx::SPI::Mode0, stm32f4xx::SPI::PRESCALER_128);
    stm32f4xx::SPI::spi3.enable();

    TaskHandle_t xHandle = NULL;
    xTaskCreate(run_main, "MAIN", 512, NULL, tskIDLE_PRIORITY, &xHandle);

    SCB->AIRCR = AIRCR_VECTKEY_MASK | NVIC_PriorityGroup_4;

   // SCnSCB->ACTLR |= 1<<1;

    vTaskStartScheduler();
}
