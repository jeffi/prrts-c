#ifndef MT19937A_H
#define MT19937A_H

/* Period parameters */  
#define MT19937A_N 624

typedef struct mt19937a {
        unsigned long mt[MT19937A_N];
        int mti;
} mt19937a_t;

void mt19937a_init_genrand(mt19937a_t *rnd, unsigned long s);
void mt19937a_init_by_array(mt19937a_t *rnd, unsigned long init_key[], int key_length);
/* generates a random number on [0,0xffffffff]-interval */
unsigned long mt19937a_genrand_int32(mt19937a_t *rnd);
/* generates a random number on [0,0x7fffffff]-interval */
long mt19937a_genrand_int31(mt19937a_t *rnd);
/* generates a random number on [0,1]-real-interval */
double mt19937a_genrand_real1(mt19937a_t *rnd);
/* generates a random number on [0,1)-real-interval */
double mt19937a_genrand_real2(mt19937a_t *rnd);
/* generates a random number on (0,1)-real-interval */
double mt19937a_genrand_real3(mt19937a_t *rnd);
/* generates a random number on [0,1) with 53-bit resolution*/
double mt19937a_genrand_res53(mt19937a_t *rnd);

#endif /* MT19937A_H */
