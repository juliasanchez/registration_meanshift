#ifndef ENVELOPE
#define ENVELOPE

#include <iostream>
#include <string>
#include "vecMed.h"

void envelope(std::vector<float> corr, int neigh, std::vector<float>* env);

#include "envelope.inl"

#endif // ENVELOPE
