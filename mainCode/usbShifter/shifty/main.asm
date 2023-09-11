/*
 * main.asm
 *
 *  Created: 1/10/2023 4:13:35 PM
 *   Author: Brandon Davis
 */ 

 /*					Used Pins
 -----------------------------------------------------------
ATMEGA used pins

PORTB - used for shifting(all pins reserved)

PORTC - 
	pc5 (pin 28) used for TWI SCL
	pc4 (pin 27) used for TWI SDA
	pc0 (pin 23) used for level button
	pc1 (pin 24) used for shifter button
	
	button change are detected using pin change interrupt

PORTD -
	PD2 (pin 4) used for external interrupt for shift change
	PD3 (pin 5) used for external interrupt for test light
	PD0 (pin 2) used for gpio intrupt for HID
	PD1 (pin 3) used to drive test LED	

FUSES:

lfuse:E2
hfuse:D7
---------------------------------------------------------------------
 */



 //
 //---------------------------------------------------------------------------
 //
 //				=================Main.asm file=====================
 //
 //---------------------------------------------------------------------------
 //

 vectorTable:
	rjmp start ; Reset Handler
	rjmp buttonStateChange ; EXTINT0 IRQ0 Handler
	rjmp buttonStateChange ; EXTINT1 IRQ1 Handler
	rjmp shiftStateChange ; PCINT0 Handler
	rjmp testButton ; PCINT1 Handler
	nop ; PCINT2 Handler
	nop ; Watchdog Timer Handler
	nop ; Timer2 Compare A Handler
	nop ; Timer2 Compare B Handler
	nop ; Timer2 Overflow Handler
	nop ; Timer1 Capture Handler
	nop ; Timer1 Compare A Handler
	nop ; Timer1 Compare B Handler
	nop ; Timer1 Overflow Handler
	nop ; Timer0 Compare A Handler
	nop ; Timer0 Compare B Handler
	nop ; Timer0 Overflow Handler
	nop ; SPI Transfer Complete Handler
	nop ; USART, RX Complete Handler
	nop ; USART, UDR Empty Handler
	nop ; USART, TX Complete Handler
	nop ; ADC Conversion Complete Handler
	nop ; EEPROM Ready Handler
	nop ; Analog Comparator Handler
	rjmp usbTWI ; 2-wire Serial Interface Handler

;end of vectorTable

;----------------------------------------------------------------------------------

;start IRQH

testButton:

	push r30
	push r31
	push r24

	ldi r30, eventLoopLow
	ldi r31, eventLoopHigh

	ld r24, z

	ori r24, addTestFlagBit

	st z, r24

	pop r24
	pop r31
	pop r30

reti

buttonStateChange:

	push r30
	push r31
	push r24

	ldi r30, eventLoopLow
	ldi r31, eventLoopHigh
	
	ld r24, z

	ori r24, addButtonPressFlagBit

	st z, r24

	pop r24
	pop r31
	pop r30

reti

shiftStateChange:

	push r30
	push r31
	push r24
	
	ldi r30, eventLoopLow
	ldi r31, eventLoopHigh

	ld r24, z

	ori r24, addShiftFlagBit

	st z, r24

	pop r24
	pop r31
	pop r30

reti

usbTWI:

	push r30
	push r31
	push r24
	
	ldi r30, eventLoopLow
	ldi r31, eventLoopHigh

	ld r24, z

	ori r24, addTWIFlagBit

	st z, r24

	ldi r30, TWCR
	clr r31
	ld r24, z
	andi r24, 0x7E ; disables twi intrupt
	st z, r24

	pop r24
	pop r31
	pop r30

reti

;end of IRQH
;--------------------------------------------------------------------------------------

start: ; Main program start
//intialize stack pointer
	ldi r16, high(RAMEND)
	out SPH,r16 
	ldi r16, low(RAMEND)
	out SPL,r16

//clear zero register
	clr r1

//start delay
	ldi r30, delayLow
	ldi r31, delayHigh

	ldi r22, 0x9A;----------------------------------
	ldi r23, 0x02;	word ? 1 millisec
	ldi r24, 0x00;
	ldi r25, 0x00;--------------------------------
	st z+, r22
	st z+, r23
	st z+, r24
	st z, r25

	rcall delayLoop

//set up i/o DD Registers

	ldi r24, ddrPortCMask
	ldi r22, ddrPortDMask

	out DDRC, r24
	out DDRD, r22

//set up extint 0
	
	ldi r30, EICRA
	clr r31
	ldi r24, anyLogicalChangeBitSet
	st z, r24

	ldi r24, enableExInt0BitSet
	out EIMSK, r24
	
//set up PCI2 and 0
	ldi r30, PCICR
	ldi r24, enablePCIBitSet
	st z, r24 ; turn on PCI1

	ldi r30, PCMSK2
	ldi r24, pinChange2BitMask ; bit mask for enable the specfic PCI1 pins
	st z, r24

	ldi r30, PCMSK0
	ldi r24, pinChange0BitMask ; bit mask for enable the specfic PCI1 pins
	st z, r24


//prepare sleep mode

	ldi r24, sleepModePowerDownMask
	out SMCR, r24


//set hid twi interface

	//fill report register with report descriptor
	ldi r24, eepromHidReportAddressOffset
	clr r25
	ldi r22, eepromHIDReportLength
	clr r23
	ldi r20, hidReportRegisterLow
	ldi r21, hidReportRegisterHigh

	rcall fillReportRegister

	//fill hid register with hid descriptor
	ldi r24, eepromHidDescriptorOffset
	clr r25
	ldi r22, eepromHidDescriptorLength
	clr r23
	ldi r20, hidDesriptorRegisterLow
	ldi r21, hidDesriptorRegisterHigh

	rcall fillReportRegister

	//init the hid driver

	rcall initHidDriver
	
	//init TWI interface
	ldi r24, TWIslaveAddress ;load r24 with TWI slave address 0x03
	clr r25

	rcall setUpI2CDevice

	//setup event flag pointer

	ldi r26, eventLoopLow
	ldi r27, eventLoopHigh
	st x, r1

	//reset all External int flags
	ldi r25, 0xff
	out EIFR, r25
	out PCIFR, r25


	sei

	;-------------------------------------------------------------------------------------------
	//the main routine acts as the primary event loop
	//eventLoop is the location of memory that stores the bit flags for each event that can occur in this program
	//the address for eventLoop is stored in the X pointer registers

main:

	ld r2, x

	//test flag remove for release
	//ldi r24, 4
	//mov r2, r24
	//^

	sbrc r2, twiFlagBitNum;check for TWI flag
	rcall mainFlagHandler

	sbrc r2, shiftFlagBitNum ; shift ocurred on the exint0 pin or pin change related to the buttons
	rcall getShiftValues
	sbrc r2, testFlagBitNum ; the test button was pressed
	rcall testButtonActivated

	
rjmp main

//test fuctions (to be removed in later versions)
//if any errors appear bring error pin high for LED indicator
errorCatchLoop:
	sbi PORTD, PortDTestLedBitNum
	daLoopOfError:
	rjmp daLoopOfError

	;TestButton creates a blocking delay in the program (to be removed in later versions)
testButtonActivated:
	//insert parameters for delay loop 
	ldi r22, 0xCF;----------------------------------
	ldi r23, 0x07;	word ? 3 millisec
	ldi r24, 0x00;
	ldi r25, 0x00;--------------------------------

	rcall delayLoop

	ld r16, x

	andi r16, removeTestFlagBit

	st x, r16

	sbic PORTD, PortDTestLedBitNum
	rjmp clearTestLedDriver
	sbi PORTD, PortDTestLedBitNum
	rjmp endOfTesDelay
	clearTestLedDriver:
	cbi PORTD, PortDTestLedBitNum

	endOfTesDelay:

ret

//end of test functions

//the pins for are H-Shifter have changed their logic levels, get the states 
//and store this in the HID input register, hold the INT line high to indicate to hid chip 
//that the host need to handle this new change

getShiftValues:
	
	//remove shift flag bit
	ld r16, x

	andi r16, removeShiftFlagBit

	st x, r16
	
	//fetch new input shift values

	
	
	in r24, PINB
	in r25, PINC

	andi r25, pincButtonMask

	rcall hidSetInputVal
	cp r24, r1
	brne failedInputHIDnew
	
	rcall hidPullIntruptLine
	cp r24, r1
	brne failedInputHIDnew

	rjmp doneWithShiftValuesGet

	//if seting input line failed (most likely due too an already existing i2c transaction going on)
	//then readd the flag until it works
	failedInputHIDnew:
		ori r16, addShiftFlagBit
		st x, r16

	doneWithShiftValuesGet:
	
ret

//The USB host has indicated the that all devices have to go in to low power mode
//put the device in low power mode until woken up again
powerDownMode:

	ldi r20, sleepBitMask

	out SMCR, r20; set sleep enable bit

	sleep

	ldi r20, sleepModePowerDownMask
	out SMCR, r20; clear sleep enable bit

ret

//a request to this slave device was made on the TWI bus, HANDLE IT HERE!
mainFlagHandler:
	
	//remove TWI flag bit
	ld r16, x

	andi r16, removeTWIFlagBit

	st x, r16
	//began the calls for a HID handle

	rcall mainHidEventHandler ; let the hid handler take care of the TWI line
	rcall hidGetErrorCode ; check to see if there is anything the main program must do from this end

	sbrc r24, resetBitErr
	rcall hidPullIntruptLine
	sbrc r24, powerBitErr 
	rcall powerDownMode ; if device is told to power down being going into low current mode

ret

//main file defines

	//exint0 pin bit defines

		.equ anyLogicalChangeBitSet = 0x05
		.equ enableExInt0BitSet = 0x03

	//enof exint0 pin bit defines

	//pin change defines

	.equ enablePCIBitSet = 0x05

		//pin change  bit masks

		.equ pinChange2BitMask = 0x08
		.equ pinChange0BitMask = 0xff
		//enof pin change bit masks

	//enof pin change  defines

	//sleep mode bit mask

	.equ sleepModePowerDownMask = 0x04
	.equ sleepBitMask = 0x05
	//enof sleep mode bit mask

	//port setup masks

	.equ ddrPortCMask = 0x0C
	.equ ddrPortDMask = 0xF3

	.equ pincButtonMask = 0x03

	//enof port setup masks

	//test LED defines


	//enof test LED defines

	.equ PortDTestLedBitNum = 1

	//TWI HID Defines

	.equ eepromHidReportAddressOffset = 0
	.equ eepromHIDReportLength = 32

	.equ eepromHidDescriptorOffset = 32
	.equ eepromHidDescriptorLength = 29

	.equ TWIslaveAddress = 0x06

	//enof TWI HID Defines

	//event flag bit defines

	.equ shiftFlagBitNum = 0
	.equ twiFlagBitNum = 2
	.equ testFlagBitNum = 3
		//event flag AND REMOVE bit masks

		.equ removeShiftFlagBit = 0xFE
		.equ removeTWIFlagBit = 0xFB
		.equ removeTestFlagBit = 0xF7
		//enof event flag AND REMOVE bit masks

		//event flag OR ADD bit mask

		.equ addShiftFlagBit = 0x81
		.equ addTWIFlagBit = 0x04
		.equ addTestFlagBit =0x08 
		.equ addButtonPressFlagBit = 0x01
		//enof event flag OR ADD bit mask

		

	//enof event flag bit defines

	//hid bit errors
	.equ resetBitErr = 0x00
	.equ powerBitErr = 0x01

	//enof hid bit errors


//enof main file defines



 //
 //---------------------------------------------------------------------------
 //
 //				=================i2cUsbHid.asm file=====================
 //
 //---------------------------------------------------------------------------
 //

 /*=============================================================
 /				Important memory locations
 /
 /	hidWriteRegister -
 /		address (little endian) = 0x2A01
 /		description = for first 2 byte I2C writes this register is what holds the address of the register to be written too.
 /		length = 2 bytes
 /
 /	hidFlag - 
 /		address = 0x2C01
 /		description = stores information of the ongoing SLA+W transaction in the bits
 /					  it is resetted upon a SLA+W status code or an end of transmission code
 /		length = 1 byte
 /
 /		bitmap =
 /				hidFlagRegister
 /	---------------------------------------------
 /	|					| CMNP | CMNCH | RP	| C |
 /	|````````````````````````````````````````````
 /	|			7-4		|  3   |   2   | 1  | 0 |
 /	---------------------------------------------
 /
 /	0 = the Counting Bit is to determine which byte in the hidWriteRegister (or other register being written too) to store too by adding this bit to the low byte pointer.
 /
 /	1 = the Register Pointer Bit is set when the counting bit is incremented after having already been set.
 /		If this bit is set then the write handler will instead store the I2C data at the register address pointed to by the hidWriteRegister.
 /
 /	2 = the Command Check Bit is set when the register is incremented with both the RP and C bits set.
 /		This bit when set tells the write handler to check if the register that was written to is the command register.
 /
 /	3 = the Command Proccess Bit is set if the write handler function determined the register written too was the command register.
 /		When this bit is set the write handler will rcall the command processing function and the write handler is called it will imeiditaly rcall that function
 /		instead of handling the writes (the command process function will handles an SLA+W based on the given HID command)
 /
 /	7-4 = not used
 /
 /	hidReadCounter -
 /		address = 0x2D01
 /		description = stores the current byte to read from into a register and write to the I2C bus during slave transmitter mode.
 /					  Is resetted by a new SLA+R transaction
 /
 /
 /	hidErrorFlag - 
 /		address = 0x2E01
 /		description = despite the name this is just used to store a code for the event loop to handle for command codes like: reset or power_set (should change name later)
 /
 /==============================================================
 */


//intialization functions of I2C HID

initHidDriver:
//prepare write register for reading HID descriptor
	ldi r30, hidWriteRegisterLow
	ldi r31, hidWriteRegisterHigh

	ldi r24, hidDesriptorRegisterLow
	ldi r25, hidDesriptorRegisterHigh

	//store write register with address of hid descriptor
	st z+, r24
	st z, r25

	//init error register to zero
	ldi r30, hidErrorFlagLow
	ldi r31, hidErrorFlagHigh

	st z, r1

	//init readCounter to zero
	ldi r30, hidReadCounterLow
	ldi r31, hidReadCounterHigh
	st z, r1

ret

 fillReportRegister://r24 eeprom address offset | r22 length of report | r20 register desanation
	
	mov r30, r20
	mov r31, r21
	
	add r22, r24 ; add the offset to the length to get the proper comparison value

	//clr r0	; used as counter
	rjmp reportLoopCompare

	//eeprom read loop
	reportEEPROMLoop:
		//add r24, r0	; add to the offset for whatever r0 is
		out EEARL, r24 ; set read address
		sbi EECR,EERE ; begin read from EEPROM
		in r25,EEDR ; get data from eeprom data

		st z+, r25	; store in dest register and increment pointer
		inc r24	; increment r0 count

	reportLoopCompare:
		cp r24, r22
		brne reportEEPROMLoop


 ret



 //end of I2C HID intialization

 //I2C HID event handler

 mainHidEventHandler:	; the hid event handler runs under the assumption that the 
	
	rcall getI2Cstatus

	//move to specfic function based on status call

	//check for  SLA+W status

	cpi r24, beginWrite
		breq statusConBeginWrite
	cpi r24, dataRecieved
		breq statusConDataRecieved
	cpi r24, eot
		breq statusConEOT
	cpi r24, datRecivendNotAcked 
		breq errorStatusNAcked
	

	rjmp codeConvertSLARbegin

	statusConBeginWrite: ; slave address has been called with a write bit reset the hid flag now
		ldi r30, hidFlagLow
		ldi r31, hidFlagHigh
		st z, r1
		rjmp mainHidEventHandlerEnd
		
	statusConDataRecieved: ; data has been recevied while in slave write mode

		rcall hidWriteHandler
		rjmp mainHidEventHandlerEnd
	statusConEOT: ; transmission ended reset hid flag for good measure
		
		rjmp statusConBeginWrite
	errorStatusNAcked:
		rcall errorCatchLoop
	

	//check for SLA+R
	codeConvertSLARbegin: 
	cpi r24, beginRead
	breq statusConBeganRead
	cpi r24, sentDataAcked
	breq statusConDataAcked
	cpi r24, sentDataNotAcked
	breq statusConDataNAcked
	rjmp mainHidEventHandlerEnd

	statusConBeganRead:
		

	statusConDataAcked:
		rcall hidReadHandler
		rjmp mainHidEventHandlerEnd
	statusConDataNAcked:
		cbi PORTD, potbGPIOintruptPinBit ; clear the gpio pin regardless of set or not
		ldi r30, hidReadCounterLow
		ldi r31, hidReadCounterHigh
		st z, r1

	
	



	mainHidEventHandlerEnd:
	
	//remove TWI intrupt flag so bus master can resume control
	//and renable global intrupts
	ldi r30, TWCR
	clr r31
	ld r24, z
	ori r24, 0x81
	st z, r24


 ret

 

 hidReadHandler:

	push r26
	push r27

	//get the register from which the read is taking place on
	ldi r30, hidWriteRegisterLow
	ldi r31, hidWriteRegisterHigh

	ld r26, z

	cpi r26, 0x2F
	brne contToRead
	
	//set to hid descriptor address high
	ldi r27, 0x01


	contToRead:

	//get read counter (to add to current address offset)
	ldi r30, hidReadCounterLow
	ldi r31, hidReadCounterHigh

	ld r0, z

	//add offset to 16-bit read address
	add r26, r0
	adc r27, r1

	//increment read counter and store back
	inc r0
	st z, r0

	//get data to transmit on the bus
	ld r24, x

	//set data to tranmit in TWI data register
	rcall writeI2Cdata

	pop r27
	pop r26

 ret


 hidWriteHandler:
	push r26
	push r27
	push r16
	push r17

	//get data that was written

	rcall readI2Cdata
	mov r0, r24

	//load the write flag into register 16
	ldi r26, hidFlagLow
	ldi r27, hidFlagHigh

	ld r16, x

	//get byte counter
	mov r17, r16
	andi r17, writeFlagCounterMask ; remove all bits except the counter bit

	//check to see if a command is being processed
	sbrc r16, writeFlagCommandBit
	rcall hidCommandProcess
	sbrs r16, writeFlagInterRegBit
	rjmp normalHidWritePPointer

		ldi r30, hidWriteRegisterLow
		ldi r31, hidWriteRegisterHigh

		ld r24, z+
		ld r25, z

	rjmp mainWritePortionOfHandler
	normalHidWritePPointer:
		ldi r24, hidWriteRegisterLow
		ldi r25, hidWriteRegisterHigh

	mainWritePortionOfHandler:
		mov r30, r24
		mov r31, r25

		add r30, r17

		st z, r0
		inc r16

	sbrs r16, writeFlagCheckCommandBit
	rjmp endOfHidWriteHandler

	cpi r24, hidCommandRegisterLow
	brne endOfHidWriteHandler
	cpi r25, hidCommandRegisterHigh
	brne endOfHidWriteHandler

	ori r16, writeFlagCommandProcMask
	

	rcall hidCommandProcess

	endOfHidWriteHandler:

	st x, r16

	pop r17
	pop r16
	pop r27
	pop r26

 ret

 hidCommandProcess:

	ldi r30, hidCommandRegisterLow
	ldi r31, hidCommandRegisterHigh

	ld r24, z+
	ld r25, z

	//comapre to opcodes
	cpi r25, resetCommandHighB
	breq resetCommandJump
	cpi r25, getReportCommandHigh
	breq getReportCommandJump
	cpi r25, setPowerCommandHigh
	breq setPowerCommandJump
	rjmp endOfCommandProc
	//jump table
	resetCommandJump:
		ldi r30, hidInputRegisterLow
		ldi r31, hidInputRegisterHigh
		//clear the input register for reset init
		st z+, r1
		st z+, r1
		st z+, r1
		st z, r1
		//set error to make sure gpio int is pulled 
		ldi r30, hidErrorFlagLow
		ldi r31, hidErrorFlagHigh

		ldi r23, forceResetError
		st z, r23
		rjmp endOfCommandProc

	getReportCommandJump:
		//incoming report read set write read register to input for reading
		ldi r30, hidWriteRegisterLow
		ldi r31, hidWriteRegisterHigh

		ldi r25, hidInputRegisterLow
		ldi r24, hidInputRegisterHigh

		st z+, r25
		st z, r24
		rjmp endOfCommandProc

	setPowerCommandJump:
		//set error to place processor into low power sleep mode
		ldi r30, hidErrorFlagLow
		ldi r31, hidErrorFlagHigh

		ldi r23, powerLowError
		st z, r23
	endOfCommandProc:


 ret


 //end of I2C HID event handlers

 //I2C HID exposed driver functions

 hidPullIntruptLine://if return in r24 is anything other then zero it failed to pull the line
	//check to make sure no on going transaction is occuring
	//get the flag value
	ldi r30, hidFlagLow
	ldi r31, hidFlagHigh
	
	ld r24, z

	cp r24, r1
	brne hidFinnishedPullUp
	
	//get read count as indication of ungoing read transaction
	ldi r30, hidReadCounterLow
	ldi r31, hidReadCounterHigh
	
	ld r24, z

	cp r24, r1
	brne hidFinnishedPullUp
	//if made it here there is no ongoign I2C transaction
	//set input register for reading in write register
	ldi r30, hidWriteRegisterLow
	ldi r31, hidWriteRegisterHigh
	ldi r22, hidInputRegisterLow
	ldi r23, hidInputRegisterHigh

	st z+, r22
	st z, r23
	//clear the error(afteraction) register
	ldi r30, hidErrorFlagLow
	ldi r31, hidErrorFlagHigh

	st z, r1
	//pull the gpio line high
	sbi PORTD, potbGPIOintruptPinBit

	mov r24, r1
	hidFinnishedPullUp:

 ret


 hidGetErrorCode:
 //set z pointer to error flag ld r24 and return that code
	ldi r30, hidErrorFlagLow
	ldi r31, hidErrorFlagHigh

	ld r24, z

 ret

 hidSetInputVal://r24-25 value to to input register
				//if return in r24 is anything other then zero it failed to pull the line

	//check to make sure no on going transaction is occuring
	//get the flag value
	ldi r30, hidFlagLow
	ldi r31, hidFlagHigh

	ld r20, z

	cp r20, r1
	brne hidFillInputRegDone
	
	//get read count as indication of ungoing read transaction
	ldi r30, hidReadCounterLow
	ldi r31, hidReadCounterHigh

	ld r20, z

	cp r20, r1
	brne hidFillInputRegDone
	
	//if made to this point no ongoing I2C HID transaction is ocurring
	ldi r30, hidInputRegisterLow
	ldi r31, hidInputRegisterHigh

	ldi r22, hidInputRegisterLength

	//store (length of report) + (report data) in input register
	st z+, r22
	st z+, r1
	st z+, r24
	st z, r25

	hidFillInputRegDone: 
	mov r24, r20
 ret

 //enof I2C HID exposed driver functions


 //hid defines

 .equ writeFlagCommandBit = 3
 .equ writeFlagCheckCommandBit = 2
 .equ writeFlagInterRegBit = 1

	//hid bit masks

	.equ writeFlagCounterMask = 0x01
	.equ writeFlagCommandProcMask = 0x08

	//end of hid bit masks

	//commands
		//the low byte on all commands contain variable data relatie to the given command opcode in the high byte
		//reset command
		
		.equ resetCommandLowB = 0x00
		.equ resetCommandHighB = 0x01
		.equ resetWriteFlagBit = 16

		//enof reset command

		//get-report command

		.equ getReportCommandLow = 0x00 ;contains report type and report id 
		.equ getReportCommandHigh = 0x02
		.equ getReportWriteFlagBit = 32

			//get-report bit masks

			.equ reportIdBitMask = 0x0F
			.equ reportTypeGetBitMask = 0x30
			//enof get-report bit masks

		//enof get-report command

		//set-report command

		.equ setReportCommandLow = 0x00 ; contains report type and report id
		.equ setReportCommandHigh = 0x03
		.equ setReportWriteFlagBit = 64

		//enof set-report command

		//set-power command

		.equ setPowerCommandLow = 0x00 ; contains power setting
		.equ setPowerCommandHigh = 0x08
		.equ setPowerWriteFlagBit =  0x80

		//enof set-power command

	//end of commands


	//error defines

	.equ forceResetError = 1
	.equ powerLowError = 2

	//enof error defines

	//pin bits

	.equ potbGPIOintruptPinBit = 0

	//enof pin bits

 //end of hid defines

 //
 //---------------------------------------------------------------------------
 //
 //				=================basicI2cDriver.asm file=====================
 //
 //---------------------------------------------------------------------------
 //


 //I2C intialization

 setUpI2CDevice: //r24 slave address to set (loads full byte into address register including general call address FYI)

	//set the two wire bit rate
	ldi r30, TWBR //setup pointer to the bit rate Two wire register
	clr r31	;clear high z pointer 

	ldi r25, twiBitMaskRate ; value of 2 for bit rate to achieve 400khz fast mode

	st z, r25

	//set the slave address
	ldi r30, TWAR

	st z, r24

	//set the control register
	ldi r30, TWCR
	ldi r25, twiControlRegMask

	st z, r25

	

 ret

 //end of I2C intialization


 //I2C driver functions

 getI2Cstatus:

	ldi r30, TWSR
	clr r31
	ld r24, z
	

	andi r24, 0xF8 ; keep only status flag

	

 ret

 readI2Cdata:

	ldi r30, TWDR
	clr r31

	ld r24, z

 ret

 writeI2Cdata:

	ldi r30, TWDR
	clr r31

	st z, r24

 ret


 //end of I2C driver functions


 //I2C setup values

 .equ twiBitMaskRate = 2
 .equ twiControlRegMask = 0x45

 //end of I2C values

 //I2C bit masks

 .equ i2cStatusRegMask = 0xF8

 //end of I2C bit masks

 //status codes

	//slave reciever
 .equ beginWrite = 0x60
 .equ dataRecieved = 0x80
 .equ datRecivendNotAcked = 0x88
 .equ eot = 0xA0
 
	//slave transmitter
 .equ beginRead = 0xA8
 .equ sentDataAcked = 0xB8
 .equ sentDataNotAcked = 0xC0


 //end of status codes

 //I2C status flags

 .equ i2cBeganSLAWriteFlag = 1
 .equ i2cDataRecievedFlag = 2
 .equ i2cEOTflag = 4

 .equ i2cBeganReadFlag = 8
 .equ i2cDataSentAckedFlag = 10
 .equ i2cDataSentNAckedFlag = 20

 //end of I2C status flags


 //I2C bit masks

 .equ removeI2CIntruptFlag = 0x7F

 //end of bit masks



 //
 //---------------------------------------------------------------------------
 //
 //				=================memMap.inc file=====================
 //
 //---------------------------------------------------------------------------
 //





 //hid memory region
 .equ sramStartOffsetLow = 0x00
 .equ sramStartOffsetHigh = 0x01
//starts at 0x0001

.equ hidReportRegisterLow = 0x00
.equ hidReportRegisterHigh = 0x01
.equ hidReportRegisterLength = 32

.equ hidCommandRegisterLow = 0x20
.equ hidCommandRegisterHigh = 0x01
.equ hidCommandRegisterLength = 2

.equ hidDataRegisterLow = 0x22
.equ hidDataRegisterHigh = 0x01
.equ hidDataRegisterLength = 4

.equ hidInputRegisterLow = 0x26
.equ hidInputRegisterHigh = 0x01
.equ hidInputRegisterLength = 4

.equ hidWriteRegisterLow = 0x2A
.equ hidWriteRegisterHigh = 0x01
.equ hidWriteRegisterLength = 2

.equ hidFlagLow = 0x2C
.equ hidFlagHigh = 0x01
.equ hidFlagLength = 1

.equ hidReadCounterLow = 0x2D
.equ hidReadCounterHigh =  0x01
.equ hidReadCounter = 1

.equ hidErrorFlagLow = 0x2E
.equ hidErrorFlagHigh =  0x01
.equ hidErrorLength = 1

.equ hidDesriptorRegisterLow = 0x2F
.equ hidDesriptorRegisterHigh =  0x01
.equ hidDescriptorLength = 30

.equ outPutRegisterLow = 0x4D
.equ outPutRegisterHigh =  0x01
.equ outputRegisterLength = 1



//end of hid memory region
//ends at 0x4E01

//eventLoop memory region
//starts at 0x4E01

.equ eventLoopLow = 0x4E
.equ eventLoopHigh =  0x01
.equ eventLoopLength = 1

//end of eventLoop region
//ends at 0x4F01

//delay memory region
//starts at 0x4F01

.equ delayLow = 0x4F
.equ delayHigh = 0x01
.equ delayMemLength = 4

//end of delay memory region
//ends at 0x5301

//delay script for button debouning  (glorifed for loop)

delayLoop:; parameters unsigned 32 bit int, lsb r22 msb r25
//delay values is calculated as follows
//(MCUhz - 10) / 12 = 1 sec loop iterations * number of secs to run = 32bit input val
	push r22
	push r23
	push r24
	push r25

	ldi r30, delayLow
	ldi r31, delayHigh
	ld r22, z+
	ld r23, z+
	ld r24, z+
	ld r25, z

	rjmp delayLoopCompare
	// 5 cycles set up

	delayLoopDec:

		subi r22, 1
		sbci r23, 0
		sbci r24, 0
		sbci r25, 0
		//4 cycles
	delayLoopCompare:
		clr r0
		or r0, r22
		or r0, r23
		or r0, r24
		or r0, r25
		cp r0, r1
		BRNE delayLoopDec
		//8 cycles total  

		//one loop is 12 cycles
	delayLoopEnd:
		
		//5 cycles (this includes the former BRNE statement) for clean 
		pop r25
		pop r24
		pop r23
		pop r22

ret

//enof button debouncer