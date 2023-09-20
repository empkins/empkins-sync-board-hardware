/*
 * m-sequence.c
 *
 *  Created on: Nov 9, 2022
 *      Author: el77enuv
 *
 *      Module providing the functionality to generate Maximal Length Sequences
 */


//includes
#include "m-sequence.h"
#include "stdio.h"
#include "main.h"
#include "math.h"


//Variables
uint8_t bit_Polynom[10][4] = { //first column = polynomial degree
		{2, 1, 0, 0}, //row 0
		{3, 2, 0, 0}, //row 1
		{4 , 3, 0, 0}, //...
		{5, 3, 0, 0},//{5, 2, 0, 0}
		{6, 5, 0, 0},
		{7, 6, 0, 0},
		{8, 4, 5, 6},// {8, 7, 6, 1}
		{9, 5, 0, 0},
		{10, 7, 0, 0},
		{11, 9, 0, 0}
};

uint8_t column = sizeof(bit_Polynom) / sizeof(bit_Polynom[0]);
uint8_t row = sizeof(bit_Polynom) / (sizeof(bit_Polynom) / sizeof(bit_Polynom[0])); //= sizeof(bit_Polynom) / column
uint8_t *out;


void lfsr(uint8_t n, uint8_t *out, uint8_t *polynom, uint8_t *tab){
	//	Function implements a linear feedback shift register
	//	taps:   Array of Polynomial exponents for non-zero terms other than 1 and n
	//	buf:    Array of buffer initialisation values as 1's and 0's or booleans
	//  out: 	Array to save the m-sequence

 	uint8_t feedback = tab[0];

	for(uint16_t i = 0; i < pow(2, n)-1; i++){
		for(uint8_t j = 0; j < row-1 ; j++){
			if(polynom[j] != 0){
				feedback = tab[0] ^ tab[polynom[j]];
				if(j == 1){
					feedback = (tab[0] ^ tab[polynom[j]]) ^ tab[polynom[j-1]];
				}else if(j == 2){
					feedback = ((tab[0] ^ tab[polynom[j]]) ^ tab[polynom[j-1]]) ^ tab[polynom[j-2]];
				}
			}
		}
		//left shift
		for(uint8_t k = 0; k < n-1; k++){
			tab[k] = tab[k+1];
		}
		tab[n-1] = feedback;
		out[i] = feedback;
	}
	return;
};

void mls(uint8_t n, uint8_t *out){
	//Generate a Maximal Length Sequence 2^n - 1 bits long
	uint8_t polynom[row - 1];//Polynoms != 1

	for(uint8_t i = 0; i < column; i++){
		if(bit_Polynom[i][0] == n){
			polynom[0] = bit_Polynom[i][1];
			polynom[1] = bit_Polynom[i][2];
			polynom[2] = bit_Polynom[i][3];
		}
	}
	//tab filled with 1s by default
	uint8_t tab[n];
	for(uint8_t i = 0; i <= n; i++){
		tab[i] = 1;
	}

	lfsr(n, out, polynom, tab);
	return;
}


//ErrorHandler

