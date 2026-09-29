/*
 * Copyright (C) 2026 NAVRobotec Pvt Ltd
 * Author: Ragnar Vallhala
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/**
 * @file physics.h
 * @brief Physical constants of the world, with one spelling each.
 *
 * Not tuning, not configuration, not a property of this airframe: values that
 * would be the same on any vehicle, so there is never a reason for two of
 * them to disagree. Anything that depends on THIS aircraft belongs in
 * control/tuning.h; anything that depends on this board belongs in
 * vayu_board.h.
 *
 * Deliberately free of every other include, so the logic layers can take a
 * constant without taking a dependency -- the reason est/vertical_estimator.h
 * kept its own copy of gravity was to avoid pulling in est/ekf.h for it.
 *
 * A duplicated physical relationship is the fault line F13 names: the same
 * formula or constant written twice across a boundary, free to diverge,
 * agreeing only by coincidence. The barometric altitude formula was one --
 * bme280.c carried its own ISA curve and its own sea-level datum beside the
 * hub's. Where a relationship has a natural owner it lives there (altitude is
 * derived once, in the hub); where it has none, it lives here.
 */
#ifndef VAYU_PHYSICS_H
#define VAYU_PHYSICS_H

/**
 * Standard gravity, m/s^2.
 */
#define VAYU_GRAVITY_MPS2 9.80665f

#endif /* VAYU_PHYSICS_H */
