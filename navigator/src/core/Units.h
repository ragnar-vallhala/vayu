#pragma once

#include <QString>

// Central display-unit + precision policy for live telemetry readouts.
//
// Settings ▸ Units & Display pushes the user's choices here once on Apply; the
// readout widgets (attitude numeric labels, the Sim HUD tapes) pull the
// formatters every render tick, so a change takes effect on the next frame with
// no signal plumbing. All inputs are SI base units (degrees / metres / m·s⁻¹);
// the helpers convert + format to the user's chosen unit at the set precision.
namespace Units {

enum class AngleUnit { Degrees, Radians };
enum class AltUnit { Meters, Feet };
enum class SpeedUnit { Mps, Kmh, Mph };

void setAngleUnit(AngleUnit u);
void setAltUnit(AltUnit u);
void setSpeedUnit(SpeedUnit u);
void setDecimals(int n);  // clamped to [0, 6]

int decimals();

// Convert a value from its SI base unit to the chosen display unit.
double toAngle(double deg);
double toAltitude(double metres);
double toSpeed(double mps);

// Unit suffix for tape titles / inline labels.
QString angleSuffix();  // "°"   | " rad"
QString altSuffix();    // "m"   | "ft"
QString speedSuffix();  // "m/s" | "km/h" | "mph"

// Converted value + suffix at the configured precision. fieldWidth pads the
// number for column alignment (0 = no padding).
QString angle(double deg, int fieldWidth = 0);
QString altitude(double metres);
QString speed(double mps);

}  // namespace Units
