
#include <stdio.h>
#include <fbm320_a18n.h>

/* Private variable */
int32_t real_p, real_t;
extern volatile uint32_t TMR0_Ticks = 0; //one tick per millisecond(ms)
extern volatile uint32_t fbm320_update_rdy = 0;

/**
 * @brief      A timer generate an interrupt every millisecond
 */
void TMR0_IRQHandler(void)
{
	if (TIMER_GetIntFlag(TIMER0) == 1)
	{
		/* Clear Timer0 time-out interrupt flag */
		TIMER_ClearIntFlag(TIMER0);

		TMR0_Ticks++;
	}
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
	/* fbm320 initiation */
	fbm320_init();

	while (1)
	{
		/* Updating fbm320 data */
		fbm320_update_data();
		if (fbm320_update_rdy) {
			
			/* This function read pressure and temperature values. Pressure uint:Pa, Temperature unit:0.01 degree Celsius */
			fbm320_read_data(&real_p, &real_t);
			fbm320_update_rdy = 0;
		}
	}
}
