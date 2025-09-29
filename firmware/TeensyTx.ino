#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdint.h>
#include <util/delay.h>
#include <usb_serial.h>
#include <DMAChannel.h>
#include <imxrt.h>

#define TEENSY_TX_2

#if defined(TEENSY_TX_1)
#define TTX_PIN_COMM_S1 6 // this must be located in port GPIO7
#define TTX_PIN_COMM_S2 7 // this must be located in port GPIO7
#define TTX_PIN_COMM_R1 8
#define TTX_PIN_COMM_R2 9
#elif defined(TEENSY_TX_2)
#define TTX_PIN_COMM_R1 37
#define TTX_PIN_COMM_R2 36
#define TTX_PIN_COMM_S1 35 // this must be located in port GPIO7
#define TTX_PIN_COMM_S2 34 // this must be located in port GPIO7
#else
#error "TEENSY_TX_1 or TEENSY_TX_2 must be defined, depending on which firmware version is being built"
#endif

#define TTX_BUF_SIZE 131072

struct FieldData
{
	unsigned char controlOp;
	unsigned char timingSamplesSetting;
	unsigned char res2;
	unsigned char res3;
	unsigned short samples[TTX_BUF_SIZE];
};

static FieldData currField;

void reboot()
{
	SCB_AIRCR = 0x05FA0004; // reboot
	for (;;)
	{
		__asm__ __volatile__("NOP");
	}
}

template <int ReleaseAhead, int FineAdjust> __attribute__((naked))
void emitSamples(unsigned short* samples)
{
	(void)samples;
	
	/* rough pseudocode:
	
	unsigned int sampleIdx = 0; // sampleIdx = r1
	unsigned int portval = 0; // portval = r2
	// GPIO6_DR is stored in r3
	unsigned int notifyIdx = TTX_BUF_SIZE - ReleaseAhead; // notifyIdx = r4
	unsigned int pinBitSet = TTX_PIN_COMM_S1; // pinBitSet = r5
	// GPIO7_DR_SET is stored in r6
	
	for (;;)
	{
		portval = (unsigned int)sampleArray[sampleIdx] << 16; // portval = r2
		
		GPIO6_DR = portval; // each "str r2, [r3]" is equivalent to GPIO6_DR = portval;
		__asm__ __volatile__("dsb" ::: "memory"); // dsb has no C equivalent... I guess
		
		++sampleIdx;
		if (sampleIdx == TTX_BUF_SIZE)
			break;
	}
	
	GPIO6_DR = 0;
	*/
	
	// Note: in certain FineAdjust paths we waste some cycles using a useless invocation of instructions like "clz"
	// this is to ensure that the loop always takes the exact number of CPU cycles, regardless of FineAdjust value
	
	__asm__ __volatile__("push    {r4-r7, lr}");
	__asm__ __volatile__("movs    r1, #0");
	__asm__ __volatile__("ldr     r3, =%c0" :: "i" (&GPIO6_DR));
	__asm__ __volatile__("ldr     r4, =%c0" :: "i" (TTX_BUF_SIZE - ReleaseAhead));
#ifdef TEENSY_TX_1
	__asm__ __volatile__("ldr     r5, =%c0" :: "i" (1 << CORE_PIN6_BIT));
#else
	__asm__ __volatile__("ldr     r5, =%c0" :: "i" (1 << CORE_PIN35_BIT));
#endif
	__asm__ __volatile__("ldr     r6, =%c0" :: "i" (&GPIO7_DR_SET));
	__asm__ __volatile__("1:");
	if constexpr (FineAdjust == 0) { __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("ldrh    r2, [r0, r1, lsl #1]");
	if constexpr (FineAdjust == 1) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("lsl    r2, r2, #16");
	if constexpr (FineAdjust == 2) { __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 3) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 4) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("dsb");
	if constexpr (FineAdjust == 5) { __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 6) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 7) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("dsb");
	if constexpr (FineAdjust == 8) { __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 9) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 10) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("str     r2, [r3]");
	if constexpr (FineAdjust == 11) { __asm__ __volatile__("clz r7, r8"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("clz r7, r7");  __asm__ __volatile__("clz r7, r7"); __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("dsb");
	if constexpr (FineAdjust == 12) { __asm__ __volatile__("cmp r1, r4"); __asm__ __volatile__("it eq"); __asm__ __volatile__("streq r5, [r6]"); }
	__asm__ __volatile__("adds    r1, r1, #1");
	__asm__ __volatile__("cmp     r1, %c0" :: "I" (TTX_BUF_SIZE));
	__asm__ __volatile__("bne     1b");
	__asm__ __volatile__("movs    r2, #0");
	__asm__ __volatile__("str     r2, [r3]");
	__asm__ __volatile__("pop     {r4-r7, pc}");
}

typedef void(*EmitSamplesFunc_t)(unsigned short*);
static EmitSamplesFunc_t emitSamplesFuncArr[256] = {
	emitSamples<1, 0>, emitSamples<1, 1>, emitSamples<1, 2>, emitSamples<1, 3>, emitSamples<1, 4>, emitSamples<1, 5>, emitSamples<1, 6>, emitSamples<1, 7>, emitSamples<1, 8>, emitSamples<1, 9>, emitSamples<1, 10>, emitSamples<1, 11>, emitSamples<1, 12>, emitSamples<1, 13>, emitSamples<1, 14>, emitSamples<1, 15>,
	emitSamples<2, 0>, emitSamples<2, 1>, emitSamples<2, 2>, emitSamples<2, 3>, emitSamples<2, 4>, emitSamples<2, 5>, emitSamples<2, 6>, emitSamples<2, 7>, emitSamples<2, 8>, emitSamples<2, 9>, emitSamples<2, 10>, emitSamples<2, 11>, emitSamples<2, 12>, emitSamples<2, 13>, emitSamples<2, 14>, emitSamples<2, 15>,
	emitSamples<3, 0>, emitSamples<3, 1>, emitSamples<3, 2>, emitSamples<3, 3>, emitSamples<3, 4>, emitSamples<3, 5>, emitSamples<3, 6>, emitSamples<3, 7>, emitSamples<3, 8>, emitSamples<3, 9>, emitSamples<3, 10>, emitSamples<3, 11>, emitSamples<3, 12>, emitSamples<3, 13>, emitSamples<3, 14>, emitSamples<3, 15>,
	emitSamples<4, 0>, emitSamples<4, 1>, emitSamples<4, 2>, emitSamples<4, 3>, emitSamples<4, 4>, emitSamples<4, 5>, emitSamples<4, 6>, emitSamples<4, 7>, emitSamples<4, 8>, emitSamples<4, 9>, emitSamples<4, 10>, emitSamples<4, 11>, emitSamples<4, 12>, emitSamples<4, 13>, emitSamples<4, 14>, emitSamples<4, 15>,
	emitSamples<5, 0>, emitSamples<5, 1>, emitSamples<5, 2>, emitSamples<5, 3>, emitSamples<5, 4>, emitSamples<5, 5>, emitSamples<5, 6>, emitSamples<5, 7>, emitSamples<5, 8>, emitSamples<5, 9>, emitSamples<5, 10>, emitSamples<5, 11>, emitSamples<5, 12>, emitSamples<5, 13>, emitSamples<5, 14>, emitSamples<5, 15>,
	emitSamples<6, 0>, emitSamples<6, 1>, emitSamples<6, 2>, emitSamples<6, 3>, emitSamples<6, 4>, emitSamples<6, 5>, emitSamples<6, 6>, emitSamples<6, 7>, emitSamples<6, 8>, emitSamples<6, 9>, emitSamples<6, 10>, emitSamples<6, 11>, emitSamples<6, 12>, emitSamples<6, 13>, emitSamples<6, 14>, emitSamples<6, 15>,
	emitSamples<7, 0>, emitSamples<7, 1>, emitSamples<7, 2>, emitSamples<7, 3>, emitSamples<7, 4>, emitSamples<7, 5>, emitSamples<7, 6>, emitSamples<7, 7>, emitSamples<7, 8>, emitSamples<7, 9>, emitSamples<7, 10>, emitSamples<7, 11>, emitSamples<7, 12>, emitSamples<7, 13>, emitSamples<7, 14>, emitSamples<7, 15>,
	emitSamples<8, 0>, emitSamples<8, 1>, emitSamples<8, 2>, emitSamples<8, 3>, emitSamples<8, 4>, emitSamples<8, 5>, emitSamples<8, 6>, emitSamples<8, 7>, emitSamples<8, 8>, emitSamples<8, 9>, emitSamples<8, 10>, emitSamples<8, 11>, emitSamples<8, 12>, emitSamples<8, 13>, emitSamples<8, 14>, emitSamples<8, 15>,
	emitSamples<9, 0>, emitSamples<9, 1>, emitSamples<9, 2>, emitSamples<9, 3>, emitSamples<9, 4>, emitSamples<9, 5>, emitSamples<9, 6>, emitSamples<9, 7>, emitSamples<9, 8>, emitSamples<9, 9>, emitSamples<9, 10>, emitSamples<9, 11>, emitSamples<9, 12>, emitSamples<9, 13>, emitSamples<9, 14>, emitSamples<9, 15>,
	emitSamples<10, 0>, emitSamples<10, 1>, emitSamples<10, 2>, emitSamples<10, 3>, emitSamples<10, 4>, emitSamples<10, 5>, emitSamples<10, 6>, emitSamples<10, 7>, emitSamples<10, 8>, emitSamples<10, 9>, emitSamples<10, 10>, emitSamples<10, 11>, emitSamples<10, 12>, emitSamples<10, 13>, emitSamples<10, 14>, emitSamples<10, 15>,
	emitSamples<11, 0>, emitSamples<11, 1>, emitSamples<11, 2>, emitSamples<11, 3>, emitSamples<11, 4>, emitSamples<11, 5>, emitSamples<11, 6>, emitSamples<11, 7>, emitSamples<11, 8>, emitSamples<11, 9>, emitSamples<11, 10>, emitSamples<11, 11>, emitSamples<11, 12>, emitSamples<11, 13>, emitSamples<11, 14>, emitSamples<11, 15>,
	emitSamples<12, 0>, emitSamples<12, 1>, emitSamples<12, 2>, emitSamples<12, 3>, emitSamples<12, 4>, emitSamples<12, 5>, emitSamples<12, 6>, emitSamples<12, 7>, emitSamples<12, 8>, emitSamples<12, 9>, emitSamples<12, 10>, emitSamples<12, 11>, emitSamples<12, 12>, emitSamples<12, 13>, emitSamples<12, 14>, emitSamples<12, 15>,
	emitSamples<13, 0>, emitSamples<13, 1>, emitSamples<13, 2>, emitSamples<13, 3>, emitSamples<13, 4>, emitSamples<13, 5>, emitSamples<13, 6>, emitSamples<13, 7>, emitSamples<13, 8>, emitSamples<13, 9>, emitSamples<13, 10>, emitSamples<13, 11>, emitSamples<13, 12>, emitSamples<13, 13>, emitSamples<13, 14>, emitSamples<13, 15>,
	emitSamples<14, 0>, emitSamples<14, 1>, emitSamples<14, 2>, emitSamples<14, 3>, emitSamples<14, 4>, emitSamples<14, 5>, emitSamples<14, 6>, emitSamples<14, 7>, emitSamples<14, 8>, emitSamples<14, 9>, emitSamples<14, 10>, emitSamples<14, 11>, emitSamples<14, 12>, emitSamples<14, 13>, emitSamples<14, 14>, emitSamples<14, 15>,
	emitSamples<15, 0>, emitSamples<15, 1>, emitSamples<15, 2>, emitSamples<15, 3>, emitSamples<15, 4>, emitSamples<15, 5>, emitSamples<15, 6>, emitSamples<15, 7>, emitSamples<15, 8>, emitSamples<15, 9>, emitSamples<15, 10>, emitSamples<15, 11>, emitSamples<15, 12>, emitSamples<15, 13>, emitSamples<15, 14>, emitSamples<15, 15>,
	emitSamples<16, 0>, emitSamples<16, 1>, emitSamples<16, 2>, emitSamples<16, 3>, emitSamples<16, 4>, emitSamples<16, 5>, emitSamples<16, 6>, emitSamples<16, 7>, emitSamples<16, 8>, emitSamples<16, 9>, emitSamples<16, 10>, emitSamples<16, 11>, emitSamples<16, 12>, emitSamples<16, 13>, emitSamples<16, 14>, emitSamples<16, 15>,
};

int main(void)
{
	pinMode(2, INPUT); // sensing pin for external power
	
	pinMode(TTX_PIN_COMM_S1, OUTPUT);
	pinMode(TTX_PIN_COMM_S2, OUTPUT);
	pinMode(TTX_PIN_COMM_R1, INPUT);
	pinMode(TTX_PIN_COMM_R2, INPUT);
	
	pinMode(41, OUTPUT);
	pinMode(40, OUTPUT);
	pinMode(39, OUTPUT);
	pinMode(38, OUTPUT);
	pinMode(27, OUTPUT);
	pinMode(26, OUTPUT);
	pinMode(23, OUTPUT);
	pinMode(22, OUTPUT);
	pinMode(21, OUTPUT);
	pinMode(20, OUTPUT);
	pinMode(19, OUTPUT);
	pinMode(18, OUTPUT);
	pinMode(17, OUTPUT);
	pinMode(16, OUTPUT);
	pinMode(15, OUTPUT);
	pinMode(14, OUTPUT);
	
	CORE_PIN41_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN40_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN39_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN38_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN27_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN26_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN23_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN22_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN21_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN20_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN19_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN18_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN17_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN16_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN15_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	CORE_PIN14_PADCONFIG = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_SPEED(3) | IOMUXC_PAD_SRE;
	
	digitalWriteFast(TTX_PIN_COMM_S1, HIGH);
	digitalWriteFast(TTX_PIN_COMM_S2, LOW);
	
	GPIO6_DR = 0;
	
	for (;;)
	{
		uint32_t startCount = ARM_DWT_CYCCNT;
		
		interrupts();
		
		digitalWrite(LED_BUILTIN, HIGH);
		
		uint32_t total = 0;
		uint32_t lastDataReceivedMs = 0;
		
		while (total < sizeof(currField))
		{
			uint32_t res = usb_serial_read((char*)&currField + total, sizeof(currField) - total);
			if (lastDataReceivedMs && !res && (systick_millis_count - lastDataReceivedMs) > 500)
			{
				total = 0;
				continue;
			}
			
			if (res > 0)
				lastDataReceivedMs = systick_millis_count;
			
			total += res;
		}
		
		digitalWrite(LED_BUILTIN, LOW);
		
		uint32_t cycCnt = (ARM_DWT_CYCCNT - startCount) / (F_CPU_ACTUAL / 1000000);
		usb_serial_write(&cycCnt, 4);
		usb_serial_flush_output();
		
		digitalWrite(LED_BUILTIN, LOW);
		
		if (currField.controlOp == 1)
		{
			reboot();
		}
	
		noInterrupts();
		
		digitalWriteFast(TTX_PIN_COMM_S1, LOW);
		
		while (digitalReadFast(TTX_PIN_COMM_R1) == LOW)
		{
			// keep waiting
		}
		
		// select the appropriate timing from the array of available functions
		emitSamplesFuncArr[currField.timingSamplesSetting](currField.samples);
	}
}
