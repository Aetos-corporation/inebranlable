/*
 * cmd_servo.c
 *
 *  Created on: Jan 23, 2023
 *      Author: clemn
 */
#include <cmd_servo.h>
#include "log.h"

#define DEFAULT_PWM_VALUE_SAFRAN  153
#define DEFAULT_PWM_VALUE_VOILE   150

/*fonction pour modifier la valeur de la PWM pour la voile
Valeurs d'entrée pour le paramètre val:
Valeurs d'entrée pour val: intervale entre 0 et 90
*/
void set_PWM_value_voile(int val){
	//entre 150 et 200
	//150->0
	//200->90
	int value_PWM = 0;
	if (val >= 0 && val <=90){
		value_PWM = (int)(((50*val)/90)+150);
		TIM1->CCR4 = value_PWM;
		PRINT("Val pwm voile: %d, val angle: %d\n", value_PWM, val);
	}
}

/*fonction pour modifier la valeur de la PWM pour du safran
Valeurs d'entrée pour le paramètre val:
tableau de valeurs : [-30, -15, -8, -2, 0, 2, 8, 15, 30]
valeurs de PWM rentrées dans TIM1->CH4: [200, 175, 165, 156, 153, 150, 142, 127, 103]
*/
void set_PWM_value_safran(int val){
	int possible_val_PWM_safran[9] = {-30, -15, -8, -2, 0, 2, 8, 15, 30};
	int value_PWM[9] = {200, 175, 165, 156, 153, 150, 142, 127, 103};
	int val_valid = 0, index = 0;

	for (int i = 0; i<9; i++){
		if(val == possible_val_PWM_safran[i] && val_valid == 0){
			val_valid = 1;
			index = i;
		}
	}

	if(val_valid == 1){
		TIM1->CCR1 = value_PWM[index];
		PRINT("Val pwm safran: %d, val angle: %d\n", value_PWM[index], val);
	}
}

//fonction qui comporte un exemple d'utilisation de la voile en balayant la plage de donnée de 0° à 90°
void execute_exemple_voile(){
	PRINT("\n\n\n====================\n");
	PRINT("Debut exemple\n");
	set_PWM_value_voile(0);
	osDelay(1000);
	set_PWM_value_voile(15);
	osDelay(1000);
	set_PWM_value_voile(30);
	osDelay(1000);
	set_PWM_value_voile(45);
	osDelay(1000);
	set_PWM_value_voile(60);
	osDelay(1000);
	set_PWM_value_voile(75);
	osDelay(1000);
	set_PWM_value_voile(90);
	osDelay(1000);
	PRINT("Fin exemple\n");
	PRINT("====================\n");
}

//fonction qui comporte un exemple d'utilisation du safran en balayant la plage de donnée de -30° à 30° en passant par tous les crans
void execute_exemple_safran(){
	PRINT("\n\n====================\n");
	PRINT("Debut exemple\n");
	set_PWM_value_safran(-30);
	osDelay(1000);
	set_PWM_value_safran(-15);
	osDelay(1000);
	set_PWM_value_safran(-8);
	osDelay(1000);
	set_PWM_value_safran(-2);
	osDelay(1000);
	set_PWM_value_safran(0);
	osDelay(1000);
	set_PWM_value_safran(2);
	osDelay(1000);
	set_PWM_value_safran(8);
	osDelay(1000);
	set_PWM_value_safran(15);
	osDelay(1000);
	set_PWM_value_safran(30);
	osDelay(1000);
	set_PWM_value_safran(0);
	osDelay(4000);
	PRINT("Fin exemple\n");
	PRINT("====================\n");
}

void startPWMTask(void const * argument){
	//démarrage des PWM
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	//initialisation des PWM dans les valeurs par défaut
	TIM1->CCR1 = DEFAULT_PWM_VALUE_SAFRAN; //safran à 0°
	TIM1->CCR4 = DEFAULT_PWM_VALUE_VOILE; //voile dans l'axe du bateau

	while(1){
		//execute_exemple_safran();
		//execute_exemple_voile();

		osDelay(100);
	}
}
