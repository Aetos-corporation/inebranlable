/*
 * navigation.h
 *
 *  Created on: 25 mai 2023
 *      Author: Arthur
 */
#ifndef NAVIGATION_H_
#define NAVIGATION_H_

// Structure
struct point{
    float x;
    float y;
};

struct vecteur{
    float x;
    float y;
};

struct cmd{
    float safran;
    float voile;
};

float calcul_norme_vecteur(struct vecteur vector);
float distance(struct point p1, struct point p2);
float conversion_angle_360_180(float angle);
bool verif_distance_bouee(struct point bouee, struct point bateau, float* seuil);
float calcul_angle_NordMatWaypoint(struct point waypoint, struct point bateau);
float calcul_angle_EstMatAvant(float nordMatAvant);
float calcul_angle_VentMatBouee(struct point bouee, struct point bateau, float ventMatAvant, float nordMatAvant);
float calcul_angle_voile(float ventMatAvant);
float calcul_angle_safran(struct point waypoint, struct point bateau, float bateau_orientation);

int decision_strategie(struct point bouee, struct point mat, float ventMatAvant, float nordMatAvant);
struct point navigation(struct point bouee, struct point bateau, int statregie, float ventMatAvant, float nordMatAvant);
struct cmd pilotage(struct point waypoint, struct point bateau, float ventMatAvant, float nordMatAvant);


#endif /* _END_NAVIGATION_H_ */
