#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ChipConfig.h"
#include "IO.h"
#include "timer.h"
#include "PWM.h"
#include "Robot.h"
#include "ADC.h"
#include "main.h"

unsigned char stateRobot;

void OperatingSystemLoop(void) {
    if (timestamp > 60000) {
        PWMSetSpeedConsigne(0, MOTEUR_DROIT);
        PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
        stateRobot = STATE_ATTENTE;
    } else {
        switch (stateRobot) {
            case STATE_ATTENTE:
                timestamp = 0;
                PWMSetSpeedConsigne(0, MOTEUR_DROIT);
                PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
                stateRobot = STATE_ATTENTE_EN_COURS;
            case STATE_ATTENTE_EN_COURS:
                if (timestamp > 1000)
                    stateRobot = STATE_AVANCE;
                break;
            case STATE_AVANCE:
                PWMSetSpeedConsigne(robotState.vitesse, MOTEUR_DROIT);
                PWMSetSpeedConsigne(robotState.vitesse, MOTEUR_GAUCHE);
                stateRobot = STATE_AVANCE_EN_COURS;
                break;
            case STATE_AVANCE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            case STATE_TOURNE_GAUCHE:
                PWMSetSpeedConsigne(robotState.vitesse, MOTEUR_DROIT);
                PWMSetSpeedConsigne(0, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_GAUCHE_EN_COURS;
                break;
            case STATE_TOURNE_GAUCHE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            case STATE_TOURNE_DROITE:
                PWMSetSpeedConsigne(0, MOTEUR_DROIT);
                PWMSetSpeedConsigne(robotState.vitesse, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_DROITE_EN_COURS;
                break;
            case STATE_TOURNE_DROITE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            case STATE_TOURNE_SUR_PLACE_GAUCHE:
                PWMSetSpeedConsigne(robotState.vitesse - 5, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-robotState.vitesse - 5, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS;
                break;
            case STATE_TOURNE_SUR_PLACE_GAUCHE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            case STATE_TOURNE_SUR_PLACE_DROITE:
                PWMSetSpeedConsigne(-robotState.vitesse - 10, MOTEUR_DROIT);
                PWMSetSpeedConsigne(robotState.vitesse - 10, MOTEUR_GAUCHE);
                stateRobot = STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS;
                break;
            case STATE_TOURNE_SUR_PLACE_DROITE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            case STATE_RECULE:
                PWMSetSpeedConsigne(-robotState.vitesse/2, MOTEUR_DROIT);
                PWMSetSpeedConsigne(-robotState.vitesse/2, MOTEUR_GAUCHE);
                stateRobot = STATE_RECULE_EN_COURS;
                break;
            case STATE_RECULE_EN_COURS:
                SetNextRobotStateInAutomaticMode();
                break;
            default:
                stateRobot = STATE_ATTENTE;
                break;
        }
    }
}
unsigned char nextStateRobot = 0;

void SetNextRobotStateInAutomaticMode() {
    unsigned char positionObstacle = PAS_D_OBSTACLE;
    //ÈDtermination de la position des obstacles en fonction des ÈÈËtlmtres
    if (robotState.distanceTelemetreDroit < 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche > 30) //Obstacle ‡droite
        positionObstacle = OBSTACLE_A_DROITE;
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche < 30) //Obstacle ‡gauche
        positionObstacle = OBSTACLE_A_GAUCHE;
    else if (robotState.distanceTelemetreCentre < 20) //Obstacle en face
        positionObstacle = OBSTACLE_EN_FACE;
        //else if (robotState.distanceTelemetreCentre < 20 &&
        //robotState.distanceTelemetreDroit < 30 &&
        //robotState.distanceTelemetreGauche < 30 )
        //positionObstacle = OBSTACLE_EN_FACE;
    else if (robotState.distanceTelemetreDroitDroit <30)
        positionObstacle = 
    else if (robotState.distanceTelemetreDroit > 30 &&
            robotState.distanceTelemetreCentre > 20 &&
            robotState.distanceTelemetreGauche > 30) //pas d?obstacle
        positionObstacle = PAS_D_OBSTACLE;
    //ÈDtermination de lÈ?tat ‡venir du robot
    if (positionObstacle == PAS_D_OBSTACLE)
        nextStateRobot = STATE_AVANCE;
    else if (positionObstacle == OBSTACLE_A_DROITE)
        nextStateRobot = STATE_TOURNE_GAUCHE;
    else if (positionObstacle == OBSTACLE_A_GAUCHE)
        nextStateRobot = STATE_TOURNE_DROITE;
    else if (positionObstacle == OBSTACLE_EN_FACE)
        if (robotState.distanceTelemetreDroit < 30 &&
                robotState.distanceTelemetreGauche < 30)
            nextStateRobot = STATE_RECULE;
        else
            nextStateRobot = STATE_TOURNE_SUR_PLACE_GAUCHE;
    //Si l?on n?est pas dans la transition de lÈ?tape en cours
    if (nextStateRobot != stateRobot - 1)
        stateRobot = nextStateRobot;
}

int main(void) {
    /***********************************************************************************************/
    //Initialisation oscillateur
    /***********************************************************************************************/
    InitOscillator();
    /***********************************************************************************************/
    // Configuration des input et output (IO)
    /***********************************************************************************************/
    InitIO();
    LED_BLANCHE_1 = 1;
    LED_BLEUE_1 = 1;
    LED_ORANGE_1 = 1;
    LED_ROUGE_1 = 1;
    LED_VERTE_1 = 1;

    BOUTON_1 = 0;

    LED_BLANCHE_2 = 1;
    LED_BLEUE_2 = 1;
    LED_ORANGE_2 = 1;
    LED_ROUGE_2 = 1;
    LED_VERTE_2 = 1;

    /***********************************************************************************************/
    // Timer
    /***********************************************************************************************/

    InitTimer1();
    InitTimer23();
    InitTimer4();

    /***********************************************************************************************/
    // Moteur
    /***********************************************************************************************/

    InitPWM();

    robotState.acceleration = 5;
    robotState.vitesse = 20;

    /***********************************************************************************************/
    // ADC
    /***********************************************************************************************/
    InitADC1();

    /***********************************************************************************************/
    // Boucle Principale
    /***********************************************************************************************/
    while (1) {
        if (ADCIsConversionFinished() == 1) {
            ADCClearConversionFinishedFlag();
            unsigned int * result = ADCGetResult();
            float volts = ((float) result [0])* 3.3 / 4096; //Convertie la distance en volt
            robotState.distanceTelemetreGaucheGauche = 34 / volts - 5;
            volts = ((float) result [1])* 3.3 / 4096; //Convertie la distance en volt
            robotState.distanceTelemetreGauche = 34 / volts - 5;
            volts = ((float) result [2])* 3.3 / 4096; //Convertie la distance en volt
            robotState.distanceTelemetreCentre = 34 / volts - 5;
            volts = ((float) result [3])* 3.3 / 4096; //Convertie la distance en volt
            robotState.distanceTelemetreDroit = 34 / volts - 5;
            volts = ((float) result [4])* 3.3 / 4096; //Convertie la distance en volt
            robotState.distanceTelemetreDroitDroit = 34 / volts - 5;

            if (robotState.distanceTelemetreGauche < 30)
                LED_BLEUE_1 = 1;
            else
                LED_BLEUE_1 = 0;
            if (robotState.distanceTelemetreDroit < 30)
                LED_ROUGE_1 = 1;
            else
                LED_ROUGE_1 = 0;
            if (robotState.distanceTelemetreCentre < 20)
                LED_ORANGE_1 = 1;
            else
                LED_ORANGE_1 = 0;
            if (robotState.distanceTelemetreGaucheGauche < 30)
                LED_BLANCHE_1 = 1;
            else
                LED_BLANCHE_1 = 0;
            if (robotState.distanceTelemetreDroitDroit < 30)
                LED_VERTE_1 = 1;
            else
                LED_VERTE_1 = 0;

        }

    }

}// fin main