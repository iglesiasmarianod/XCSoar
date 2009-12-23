/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000, 2001, 2002, 2003, 2004, 2005, 2006, 2007, 2008, 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>
	Tobias Bieniek <tobias.bieniek@gmx.de>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "MacCready.h"
#include "Polar/Polar.hpp"
#include "SettingsComputer.hpp"
#include "Math/FastMath.h"
#include "Math/Geometry.hpp"
#include "Math/Constants.h"
#include "Thread/Mutex.hpp"

#include <tchar.h>
#include <math.h>
#include <windows.h>

/*
  double a = -0.00190772449;
  double b = 0.06724332;
  double c = -1.141438761599;
*/

//double oldGlidePolar::BallastFactor;
double oldGlidePolar::RiskGamma = 0.0;
double oldGlidePolar::PolarCoefA;
double oldGlidePolar::PolarCoefB;
double oldGlidePolar::PolarCoefC;
int oldGlidePolar::Vminsink = 2;
int oldGlidePolar::Vbestld = 2;
double oldGlidePolar::sinkratecache[MAXSAFETYSPEED];
double oldGlidePolar::bestld = 0.0;
double oldGlidePolar::minsink = 10000.0;
double oldGlidePolar::BallastLitres = 0.0;
double oldGlidePolar::WingLoading = 0.0;

double oldGlidePolar::SafetyMacCready = 0.0;
bool oldGlidePolar::AbortSafetyUseCurrent = false;
int oldGlidePolar::MAXSPEED = 0;

static int iSAFETYSPEED = 0;

static Mutex mutexoldGlidePolar;

//Flight Data Globals
double oldGlidePolar::_MacCready = 0; // JMW now in SI units (m/s) for consistency
double oldGlidePolar::_Bugs = 1;
double oldGlidePolar::_Ballast = 0;
double oldGlidePolar::_CruiseEfficiency = 1.0;

Polar polar;

void
oldGlidePolar::Lock()
{
  mutexoldGlidePolar.Lock();
}

void
oldGlidePolar::Unlock()
{
  mutexoldGlidePolar.Unlock();
}

double
oldGlidePolar::AbortSafetyMacCready()
{
  double retval;

  Lock();

  if (AbortSafetyUseCurrent)
    retval = _MacCready;
  else
    retval = SafetyMacCready;

  Unlock();

  return retval;
}

double
oldGlidePolar::GetAUW()
{
  double retval;

  Lock();
  retval = BallastLitres + polar.WEIGHTS[0] + polar.WEIGHTS[1];
  Unlock();

  return retval;
}

/**
 * Returns the chosen MacCready value
 * @return The chosen MacCready value
 */
double
oldGlidePolar::GetMacCready()
{
  double retval;

  Lock();
  retval = _MacCready;
  Unlock();

  return retval;
}

/**
 * Returns the bugs factor
 * @return The bugs factor
 */
double
oldGlidePolar::GetBugs()
{
  double retval;

  Lock();
  retval = _Bugs;
  Unlock();

  return retval;
}

/**
 * Returns the ballast percentage
 * @return The ballast percentage
 */
double
oldGlidePolar::GetBallast()
{
  double retval;

  Lock();
  retval = _Ballast;
  Unlock();

  return retval;
}

/**
 * Returns the cruise efficiency
 * @return The cruise efficiency
 */
double
oldGlidePolar::GetCruiseEfficiency()
{
  double retval;

  Lock();
  retval = _CruiseEfficiency;
  Unlock();

  return retval;
}

/**
 * Returns the ballast in liters
 * @return The ballast in liters
 */
double
oldGlidePolar::GetBallastLitres()
{
  double retval;

  Lock();
  retval = BallastLitres;
  Unlock();

  return retval;
}

/**
 * Sets the MacCready value to val
 * @param val The new MacCready value
 */
void
oldGlidePolar::SetMacCready(double val)
{
  Lock();
  _MacCready = val;
  Unlock();
}

/**
 * Sets the bugs factor to val
 * @param val The new bugs factor
 */
void
oldGlidePolar::SetBugs(double val)
{
  Lock();
  _Bugs = val;
  Unlock();
}

/**
 * Sets the cruise efficiency to val
 * @param val The new cruise efficiency
 */
void
oldGlidePolar::SetCruiseEfficiency(double val)
{
  Lock();
  _CruiseEfficiency = val;
  Unlock();
}

/**
 * Sets the ballast to val
 * @param val The new ballast percentage
 */
void
oldGlidePolar::SetBallast(double val)
{
  Lock();
  _Ballast = val;
  Unlock();
}

/**
 * Finds polar-related data (bestLD and minSinkRate)
 * and fills the sinkratecache
 * @param send
 * @param settings
 */

void
oldGlidePolar::UpdatePolar(bool send, const SETTINGS_COMPUTER &settings)
{
  Lock();

  double BallastWeight;
  // Calculate ballast in liters (=BallastLitres)
  BallastLitres = polar.WEIGHTS[2] * _Ballast;
  // Calculate total weight of the plane (=BallastWeight)
  BallastWeight = GetAUW();
  // Calculate WingLoading if possible
  if (polar.WingArea > 0.1)
    WingLoading = BallastWeight / polar.WingArea;
  else
    WingLoading = 0;

  // Correct polar for BallastWeight and bugfactor
  BallastWeight = (double)sqrt(BallastWeight);
  double bugfactor = 1.0 / _Bugs;
  PolarCoefA = polar.POLAR[0] / BallastWeight * bugfactor;
  PolarCoefB = polar.POLAR[1] * bugfactor;
  PolarCoefC = polar.POLAR[2] * BallastWeight * bugfactor;

  // do preliminary scan to find min sink and best LD
  // this speeds up maccready calculations because we have a reduced range
  // to search across.
  // this also limits speed to fly to logical values (will never try
  // to fly slower than min sink speed)

  minsink = 10000.0;
  bestld = 0.0;
  int i;

  // Limit the processed speed to the user-specified safety speed
  if ((settings.SafetySpeed == 0) || (settings.SafetySpeed >= MAXSAFETYSPEED))
    iSAFETYSPEED = MAXSAFETYSPEED - 1;
  else
    iSAFETYSPEED = (int)settings.SafetySpeed;

  MAXSPEED = iSAFETYSPEED;

  // Iterate through the speed points on the polar
  for (i = 4; i <= MAXSPEED; i++) {
    // vtrack = TAS along bearing in cruise
    double vtrack = (double)i;

    // saves the sinkrate for the current speed (vtrack) without
    // any wind or maccready
    double thesinkrate = -SinkRate(PolarCoefA, PolarCoefB, PolarCoefC, 0, 0, vtrack);

    // calculate the LD
    double ld = vtrack / thesinkrate;

    // if (LD is better then the best one found yet) save LD and speed;
    if (ld >= bestld) {
      bestld = ld;
      Vbestld = i;
    }

    // if (sinkrate is better then the best one found yet) save sinkrate and speed;
    if (thesinkrate <= minsink) {
      minsink = thesinkrate;
      Vminsink = i;
    }

    // save couple in cache
    sinkratecache[i] = -thesinkrate;
  }

  /*
  int polar_ai = iround((PolarCoefA*10)*4096);
  int polar_bi = iround((PolarCoefB)*4096);
  int polar_ci = iround((PolarCoefC/10)*4096);
  int minsinki = -iround(minsink*10);
  int vbestldi = iround(Vbestld*10);
  int bestldi = iround(bestld*10);
  */

  Unlock();

  // TODO JMW: use this instead? etc
  /*
  AllDevicesPutBugs(_Bugs);

  if (GPS_INFO.VarioAvailable) {

    TCHAR nmeabuf[100];
    _stprintf(nmeabuf,TEXT("PDVGP,%d,%d,%d,%d,%d,%d,0"),
              polar_ai,
              polar_bi,
              polar_ci,
              minsinki,
              vbestldi,
              bestldi);

    VarioWriteNMEA(nmeabuf);
  }
  */

  // TODO JMW: should call GCE_POLAR_CHANGED
  /*
  if (send) {
    AllDevicesPutBallast(_Ballast);
    AllDevicesPutBugs(_Bugs);
  }
  */
}

/**
 * Fetches the corresponding sinkrate from the sinkratecache
 * and corrects it for the MacCready value
 * @param MC MacCready value
 * @param v The speed used for calculation
 * @return The corresponding sinkrate
 */
inline double
oldGlidePolar::_SinkRateFast(const double &MC, const int &v)
{
  return sinkratecache[v]-MC;
}

/**
 * Calls _SinkRateFast with corrected speed
 * @param MC MacCready value
 * @param v The speed used for calculation
 * @return The corresponding sinkrate
 */
double
oldGlidePolar::SinkRateFast(const double &MC, const int &v)
{
  return _SinkRateFast(MC, max(4,min(iSAFETYSPEED, v)));
}

/**
 * Calculates the sinkrate for the speed V
 * @param V The speed used for calculation
 * @return The corresponding sinkrate
 * @see SinkRateFast
 */

double
oldGlidePolar::SinkRate(double V)
{
  return SinkRate(PolarCoefA, PolarCoefB, PolarCoefC, 0.0, 0.0, V);
}

#define MIN_MACCREADY 0.000000000001

/**
 * Calculates the sinkrate for the speed V and the G load n
 * @param V The speed used for calculation
 * @param n G load
 * @return The corresponding sinkrate
 */
double
oldGlidePolar::SinkRate(double V, double n)
{
  n = max(0.1, fabs(n));

  double w0 = SinkRate(PolarCoefA, PolarCoefB, PolarCoefC, 0.0, 0.0, V);
  //double v1 = V / max(1, Vbestld);
  double v2 = Vbestld / max(Vbestld/2,V);
  return w0 - (V / (2 * bestld)) * (n * n - 1) * (v2 * v2);
}

/**
 * Calculates the sinkrate for the given parameters
 * @param a Polar factor A
 * @param b Polar factor
 * @param c Polar factor
 * @param MC MacCready value
 * @param HW Head wind speed
 * @param V Plane speed
 * @return The corresponding sinkrate
 */
double
oldGlidePolar::MacCreadyAltitude_internal(double eMacCready, double Distance,
    double Bearing, const double WindSpeed, const double WindBearing,
    double *BestCruiseTrack, double *VMacCready, const bool isFinalGlide,
    double *TimeToGo, const double cruise_efficiency)
{
  int i;
  double BestSpeed, BestGlide, Glide;
  double BestSinkRate, TimeToDestCruise;
  static double HeadWind, CrossWind = 0.0;
  static double CrossBearingLast = -1.0;
  static double WindSpeedLast = -1.0;
  double CrossBearing;
  double BestTime;
  static double HeadWindSqd, CrossWindSqd = 0.0;

  CrossBearing = AngleLimit360(Bearing - WindBearing);
  if ((CrossBearing != CrossBearingLast) || (WindSpeed != WindSpeedLast)) {
    // saves a few floating point operations
    HeadWind = WindSpeed * fastcosine(CrossBearing);
    CrossWind = WindSpeed * fastsine(CrossBearing);
    HeadWindSqd = HeadWind * HeadWind;
    CrossWindSqd = CrossWind * CrossWind;

    // save old values
    CrossBearingLast = CrossBearing;
    WindSpeedLast = WindSpeed;
  }

  double sinkrate;
  double tc; // time spent in cruise

  // TODO accuracy: extensions to Mc to incorporate real-life issues
  // - [done] best cruise track and bearing (final glide and for waypoint)
  // - climb before or after turning waypoints.
  // - maccready ring changes with height allowing for risk and decreased rate
  // - cloud streets
  // - sink rate between thermals
  // - modify Vtrack for IAS

  //Calculate Best Glide Speed
  BestSpeed = 2;
  BestGlide = 10000;
  BestTime = 1e6;

  if (BestCruiseTrack)
    *BestCruiseTrack = Bearing;

  double vtot;
  if (Distance < 1.0)
    Distance = 1;

  double TimeToDestTotal = ERROR_TIME; // initialise to error value
  TimeToDestCruise = -1; // initialise to error value

  for (i = Vminsink; i < iSAFETYSPEED; i++) {
    double vtrack_real = ((double)i); // actual airspeed
    double vtrack = vtrack_real * cruise_efficiency;
    // TAS along bearing in cruise

    // glide angle = velocity projected along path / sink rate
    // need to work out best velocity along path given wind vector
    // need to work out percent time spent cruising
    // SinkRate function returns negative value for sink

    if (isFinalGlide) {
      sinkrate = -_SinkRateFast(max(0.0,eMacCready), i);
      tc = 1.0; // assume no circling, e.g. final glide at best LD
      // with no climbs
    } else {
      eMacCready = max(MIN_MACCREADY,eMacCready);
      sinkrate = -_SinkRateFast(0.0, i);
      tc = max(0.0,min(1.0,eMacCready/(sinkrate+eMacCready)));
    }

    // calculate average speed along track relative to wind
    vtot = (vtrack * vtrack * tc * tc - CrossWindSqd);

    // if able to advance against crosswind
    if (vtot > 0) {
      // if able to advance against headwind
      if (vtot > HeadWindSqd) {
        // calculate average speed along track relative to ground
        vtot = sqrt(vtot) - HeadWind;
      } else {
        // can't advance at this speed
        continue;
      }
    }

    // can't advance at this speed
    if (vtot <= 0)
      continue;

    bool bestfound = false;

    if (isFinalGlide) {
      // inverse glide ratio relative to ground
      Glide = sinkrate / vtot;

      // best glide angle when in final glide
      if (Glide <= BestGlide) {
        bestfound = true;
        BestGlide = Glide;
        TimeToDestTotal = Distance / vtot;
      }
    } else {
      // time spent in cruise
      double Time_cruise = (tc / vtot) * Distance;
      double Time_climb = sinkrate * (Time_cruise / eMacCready);

      // total time to destination
      TimeToDestTotal = max(Time_cruise+Time_climb,0.0001);
      // best average speed when in maintaining height mode
      if (TimeToDestTotal <= BestTime) {
        bestfound = true;
        BestTime = TimeToDestTotal;
      }
    }

    if (bestfound) {
      BestSpeed = min(MAXSPEED, vtrack_real);
      if (BestCruiseTrack) {
        // best track bearing is the track along cruise that
        // compensates for the drift during climb
        *BestCruiseTrack = atan2(CrossWind * (tc - 1), vtot + HeadWind * (1
            - tc)) * RAD_TO_DEG + Bearing;
      }
      if (VMacCready) {
        *VMacCready = BestSpeed;
      }

      // speed along track during cruise component
      TimeToDestCruise = Distance * tc / vtot;
    } else {
      // no need to continue search, max already found..
      break;
    }
  }

  BestSinkRate = SinkRateFast(0, (int)BestSpeed);

  if (TimeToGo) {
    *TimeToGo = TimeToDestTotal;
  }

  // this is the altitude needed to final glide to destination
  return -BestSinkRate * TimeToDestCruise;
}

double
oldGlidePolar::SinkRate(double a, double b, double c,
    double MC, double HW, double V)
{
  double temp;

  // Quadratic form: w = c+b*(V)+a*V*V

  temp = a * (V + HW) * (V + HW);
  temp += b * (V + HW);
  temp += c - MC;

  return temp;
}

/**
 * find the highest speed that provides a sink rate less than
 * the specified sink rate
 * @param w Sink rate limit
 * @return highest speed that provides a sink rate less than
 * the specified sink rate
 */
double
oldGlidePolar::FindSpeedForSinkRate(double w)
{
  double vbest = Vminsink;

  for (int v = 4; v < iSAFETYSPEED; v++) {
    double wthis = _SinkRateFast(0, v);
    if (wthis < w)
      vbest = v;
  }

  return vbest;
}

double
oldGlidePolar::MacCreadyAltitude_heightadjust(double eMacCready, double Distance,
    double Bearing, const double WindSpeed, const double WindBearing,
    double *BestCruiseTrack, double *VMacCready, const bool isFinalGlide,
    double *TimeToGo, const double AltitudeAboveTarget,
    const double cruise_efficiency)
{
  double Altitude;
  double TTG = 0;

  if (!isFinalGlide || (AltitudeAboveTarget <= 0)) {
    // if not in final glide or below target altitude, need to
    // climb-cruise the whole way
    Altitude = MacCreadyAltitude_internal(eMacCready, Distance, Bearing,
        WindSpeed, WindBearing, BestCruiseTrack, VMacCready, false, &TTG,
        cruise_efficiency);

  } else {
    // if final glide mode and can final glide part way
    double t_t = ERROR_TIME;
    double h_t = MacCreadyAltitude_internal(eMacCready, Distance, Bearing,
        WindSpeed, WindBearing, BestCruiseTrack, VMacCready, true, &t_t,
        cruise_efficiency);

    if (h_t <= 0) {
      // error condition, no distance to travel
      TTG = t_t;
      Altitude = 0;
    } else {
      double h_f = AltitudeAboveTarget;
      // fraction of leg that can be final glided
      double f = min(1.0, max(0.0, h_f / h_t));

      if (f < 1.0) {
        // if need to climb-cruise part of the way
        double d_c = Distance * (1.0 - f);
        double t_c;
        double h_c = MacCreadyAltitude_internal(eMacCready, d_c, Bearing,
            WindSpeed, WindBearing, BestCruiseTrack, VMacCready, false, &t_c,
            cruise_efficiency);

        if (h_c < 0) {
          // impossible at this Mc, so must be final glided
          Altitude = -1;
          TTG = ERROR_TIME;
        } else {
          Altitude = f * h_t + h_c;
          TTG = f * t_t + t_c;
        }
      } else {
        // can final glide the whole way
        Altitude = h_t;
        TTG = t_t;
      }
    }
  }

  if (TimeToGo) {
    *TimeToGo = TTG;
  }

  return Altitude;
}

double
oldGlidePolar::MacCreadyAltitude(double eMacCready, double Distance,
    const double Bearing, const double WindSpeed, const double WindBearing,
    double *BestCruiseTrack, double *VMacCready, const bool isFinalGlide,
    double *TimeToGo, const double AltitudeAboveTarget,
    const double cruise_efficiency)
{

  double TTG = ERROR_TIME;
  double Altitude = -1;
  bool invalidMc = (eMacCready < MIN_MACCREADY);
  bool invalidAltitude = false;

  if (!invalidMc || isFinalGlide) {
    Altitude = MacCreadyAltitude_heightadjust(eMacCready, Distance, Bearing,
        WindSpeed, WindBearing, BestCruiseTrack, VMacCready, isFinalGlide,
        &TTG, AltitudeAboveTarget, cruise_efficiency);

    if (Altitude < 0) {
      invalidAltitude = true;
    } else {
      // All ok
      if (TTG < 0.9 * ERROR_TIME)
        goto onExit;
    }
  }

  // Never going to make it at this rate, so assume final glide
  // with no climb
  // This can occur if can't make progress against headwind,
  // or if Mc is too small

  Altitude = MacCreadyAltitude_heightadjust(eMacCready, Distance, Bearing,
      WindSpeed, WindBearing, BestCruiseTrack, VMacCready, true, &TTG, 1.0e6,
      cruise_efficiency);

  if (invalidAltitude)
    // if it failed due to invalid Mc, need to increase estimated
    // time and the glider better find that lift magically
    TTG += ERROR_TIME;

onExit:
  if (TimeToGo)
    *TimeToGo = TTG;

  return Altitude;
}

static double
FRiskFunction(double x, double k)
{
  return 2.0 / (1.0 + exp(-x * k)) - 1.0;
}

double
oldGlidePolar::MacCreadyRisk(double HeightAboveTerrain, double MaxThermalHeight,
    double MacCready)
{
  double RiskMC = MacCready;

  double hthis = max(1.0, HeightAboveTerrain);
  double hmax = max(hthis, MaxThermalHeight);
  double x = hthis / hmax;
  double f;

  if (RiskGamma < 0.1)
    return MacCready;

  if (RiskGamma > 0.9)
    f = x;
  else {
    double k;
    k = 1.0 / (RiskGamma * RiskGamma) - 1.0;
    f = FRiskFunction(x, k) / FRiskFunction(1.0, k);
  }

  double mmin = 0; // min(MacCready, AbortSafetyMacCready());
  RiskMC = f * RiskMC + (1 - f) * mmin;

  return RiskMC;
}
