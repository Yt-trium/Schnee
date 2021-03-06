#ifndef ORIENTATIONFIXER_H
#define ORIENTATIONFIXER_H

#include <cmath>

#include "vector.h"
#include "cloud.h"
#include "plane.h"
#include "kruskal.h"

/**
 * @brief orientationFixer
 * @param plc
 * @param index
 * @param k
 */
void orientationFixer(PlaneCloud &plc, const plane_cloud_index &index, const int &k);

/**
 * @brief maxZ return the id of the plane with the maximum value in Z
 * @param plc
 * @return
 */
size_t maxZ(const PlaneCloud &plc);

/**
 * @brief flip
 * @param v
 */
void flip(sVector3 &v);

/**
 * @brief depthSearchFix
 * @param r : the MST
 * @param plc : the plane cloud
 * @param current : the current plane
 * @param from : the plane who call us
 */
void depthSearchFix(const std::vector<Kruskal::KEdge> &r, const PlaneCloud &plc, size_t current, size_t from);

/**
 * @brief getNBH
 * @param r
 * @param current
 * @return
 */
std::vector<size_t> getNBH(const std::vector<Kruskal::KEdge> &r, size_t current);

#endif // ORIENTATIONFIXER_H
