/**
  ******************************************************************************
  * File: ReactionTester.c
  * Description: Main program body
  ******************************************************************************
  * Attention:
  *
  * Copyright (c) 2023 BYU-Idaho
  * All rights reserved.
  *
  * Copyright: BYU-Idaho
  * Date: 2023
  * Version: F23
  * Note: For course ECEN-361
  * Author: Lynn Watson
  ******************************************************************************
  */
/* This is for Part-3 of Lab-02, ECEN-361
 * Student to only change parts between the comment blocks:
	  ***** STUDENT TO FILL IN START
 *
 */

#include "main.h"
#include "stm32l4xx_it.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include "MultiFunctionShield.h"

extern TIM_HandleTypeDef htim3;  // Points to the timer structure   Timer3 is the Reaction Timer
extern void MX_TIM3_Init(void);	 // To reset the timer
extern bool got_start_button;
extern bool got_stop_button;
extern bool got_fastest_button;
extern int best_reaction_time_in_millisec;

int rand_millisec;
int last_reaction_time_in_millisec = 0;
bool started_doing_reaction_timers = false;

void show_a_random_number()
{
    if (!started_doing_reaction_timers)
    {
        rand_millisec = rand() % 7000; // Use 7000 as upper limit as per original define
        MultiFunctionShield_Display(rand_millisec);
        HAL_Delay(2000);  // This is how long before the counter on the 7-Seg display
    }
}

void got_start()
{
    /* Here's the code to do when the Start Button is pushed
       When Start is pressed:
       1.) Display the Waiting "----"
       2.) Wait for a random number of millisec's
       3.) Turn on all the 7-Seg lights (that's "GO")
       4.) Start the Reaction timer.
    */
    started_doing_reaction_timers = true;
    Clear_LEDs();
    rand_millisec = rand() % 7000;

    /**************** STUDENT TO FILL IN START HERE ********************/
    // Step 1: Display the Waiting "----"
    MultiFunctionShield_Display(-1);

    // Step 2: Wait for a random number of milliseconds
    HAL_Delay(rand_millisec);

    // Step 3: Turn on all the 7-Seg lights to "8888" (GO signal)
    MultiFunctionShield_Display(8888);

    // Step 4: Start the reaction timer (TIM3)
    __HAL_TIM_SET_COUNTER(&htim3, 0);  // Reset TIM3 to 0
    HAL_TIM_Base_Start(&htim3);        // Start TIM3
    /**************** STUDENT TO FILL IN END  HERE ********************/
}

void got_stop()
{
    /* Here's the code for the STOP button --
     * When pushed:
     1) Stop timer
     2) Read value of timer
     3) Showvalue
     */
    if (started_doing_reaction_timers)
    {
        /**************** STUDENT TO FILL IN START HERE ********************/
        // 1.) Stop  timer (TIM3)
        HAL_TIM_Base_Stop(&htim3);

        // 2.) Read timer -- this step provided
        last_reaction_time_in_millisec = __HAL_TIM_GetCounter(&htim3) / 10; // As per original code

        // 3.) Display value
        MultiFunctionShield_Display(last_reaction_time_in_millisec);
        /**************** STUDENT TO FILL IN END HERE ********************/

        // Keep the best time in variable
        if (last_reaction_time_in_millisec < best_reaction_time_in_millisec)
        {
            best_reaction_time_in_millisec = last_reaction_time_in_millisec;
        }

        // Show stats
        printf("Random Delay was: %d\n\r", rand_millisec);
        printf("Reaction Time from Timer   : %d\n\r", last_reaction_time_in_millisec);

        // Just to keep things random -- reseed with the last reaction time
        srand((unsigned) last_reaction_time_in_millisec);

        started_doing_reaction_timers = false;
    }
}

void got_fastest()
{
    got_fastest_button = false;
    MultiFunctionShield_Display(best_reaction_time_in_millisec);
}
