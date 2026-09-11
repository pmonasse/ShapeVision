// SPDX-License-Identifier: MPL-2.0
/**
 * @file sample_ll.h
 * @brief Sampling of level line from an image
 * 
 * (C) 2026 Pascal Monasse <pascal.monasse@enpc.fr>
 */

#ifndef SAMPLE_LL_H
#define SAMPLE_LL_H

#include "cc.h"

/// Sample level line in a continuum.
std::vector<DPoint> sample_ll(const Continuum& ctn, float level,
                              const CC& cc, const float* data,
                              int ptsPixel);

#endif
