/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

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

#include "Device/Driver/JRVario.hpp"
#include "Device/Driver.hpp"
#include "Device/Port/Port.hpp"
#include "Device/Util/NMEAWriter.hpp"
#include "Operation/Operation.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "NMEA/Info.hpp"
#include "NMEA/InputLine.hpp"
#include "NMEA/Checksum.hpp"
#include "Units/Units.hpp"
#include "Dialogs/Device/DeviceListDialog.hpp"
#include "Device/device.hpp"
#include "Device/MultipleDevices.hpp"
#include "Device/Descriptor.hpp"
#include "Components.hpp"
#include "Operation/PopupOperationEnvironment.hpp"
#include "Input/InputEvents.hpp"

/*
void
JRSendNMEAPort1(const TCHAR *misc)
{
  const unsigned i = 0;

  if (misc != NULL && i < NUMDEV) {
    PopupOperationEnvironment env;
    (*devices)[i].WriteNMEA(misc, env);
  }
}
*/

/*
$PJRC,<1>,<2>,<3>,<4>*hh<CR><LF>
Commands from JRVario to XCSoar and OpenVario
<1>  MC Setting
<2>  Ballast Setting % of max ballast
<3>  Bug Setting %
<4>  Volume Setting 0 No change, 1 Volume Up, 2 Volume Down
*/

static bool
PJRC(NMEAInputLine &line, NMEAInfo &info)
{
  double i;
  char buffer[30];

  if (line.ReadChecked(i))
  {
    //Provide MC value to XCSoar and send it to OpenVario using Openvario's protocol.
    info.settings.ProvideMacCready(i,info.clock);
    sprintf(buffer,"POV,C,MC,%0.2f", i);
    InputEvents::eventSendNMEAPort1(buffer);
  }

  if (line.ReadChecked(i))
    info.settings.ProvideBallastFraction(i, info.clock);

  if (line.ReadChecked(i))
    info.settings.ProvideBugs(i, info.clock);

  if (line.ReadChecked(i))
     info.settings.ProvideQNH(AtmosphericPressure::HectoPascal(i), info.clock);

  int VolumeComand;

  if (line.ReadChecked(VolumeComand))
  {
    //Send Volume changes to OpenVario using OpenVario protocol
    if(VolumeComand==1)
      InputEvents::eventSendNMEAPort1("POV,C,VU");

    if(VolumeComand==2)
      InputEvents::eventSendNMEAPort1("POV,C,VD");
  }
  return true;
}

/*
$PJRD,<1>,<2>,<3>*hh<CR><LF>
Calculated data from XCSoar to JRVario
<1>  30s Thermal Average
<2>  Calculated STF
<3>  IAS
*/
static bool
FormatPJRD(char *buffer, size_t buffer_size, const double &average,
           const double &STF, const double &IAS)
{

    snprintf(buffer, buffer_size,
           "PJRD,%+4.1f,%.2f,%.2f",
           average,STF,IAS);

  return true;
}

/*
$PJRG,E,<1>,H,<2>*hh<CR><LF>
Sensor data from XCSoar to JRVario
<1>  Vario_Brutto
<2>  Navigation Altitude
*/
static bool
FormatPJRG(char *buffer, size_t buffer_size, const double &vario,const double &nav_altitude)
{

    snprintf(buffer, buffer_size,
           "PJRG,E,%+4.1f,H,%0.0f",
           vario,nav_altitude);

  return true;
}

class JRVario final : public AbstractDevice {
  Port &port;

public:
  JRVario(Port &_port):port(_port) {}

  /* virtual methods from class Device */

  void OnSensorUpdate(const MoreData &basic) override;

  void OnCalculatedUpdate(const MoreData &basic, const DerivedInfo &calculated) override;

  bool PutMacCready(double mc, OperationEnvironment &env) override;

  bool PutBallast(double fraction, double overload,
                    OperationEnvironment &env) override;

  bool PutQNH(const AtmosphericPressure &pres,
                         OperationEnvironment &env) override;

  bool PutBugs(double bugs, OperationEnvironment &env) override;

  bool ParseNMEA(const char *String, struct NMEAInfo &info) override;

};


void
JRVario::OnSensorUpdate(const MoreData &basic)
{
  NullOperationEnvironment env;
  char buffer[100];

    const double vario=basic.brutto_vario;
    const double nav_altitude = basic.nav_altitude;
    if (FormatPJRG(buffer, sizeof(buffer), vario,nav_altitude))
    PortWriteNMEA(port, buffer, env);

}


void
JRVario::OnCalculatedUpdate(const MoreData &basic,
                                  const DerivedInfo &calculated)
{
  NullOperationEnvironment env;
  char buffer[100];

    //const OneClimbInfo &data=calculated.current_thermal;
    //const double average = data.lift_rate;
    const double average = calculated.average;
    const double STF = calculated.V_stf;
    //const double IAS = basic.true_airspeed;
    const double IAS = basic.indicated_airspeed;

    if (FormatPJRD(buffer, sizeof(buffer), average, STF, IAS))
    PortWriteNMEA(port, buffer, env);

}
/*
$POVG,C,<command>,<value>,*hh<CR><LF>
Commands sent from XCSoar to JRVario
<command> MC:Change MC setting
          BA:Change Ballast Fraction
          QNH: Set QNH
          BU: Change Bugs Settings
 */
bool
JRVario::PutMacCready(double mc, OperationEnvironment &env)
{
  if (!EnableNMEA(env))
    return false;

  char buffer[30];
  sprintf(buffer,"POVG,C,MC,%0.2f", (double)mc);
  PortWriteNMEA(port, buffer, env);
  return true;
}

bool
JRVario::PutBallast(double fraction, double overload,
                           OperationEnvironment &env)
{
  if (!EnableNMEA(env))
    return false;

  char buffer[30];
  sprintf(buffer,"POVG,C,BA,%3f", fraction);
  PortWriteNMEA(port, buffer, env);
  return true;
}

bool
JRVario::PutQNH(const AtmosphericPressure &pres,
                       OperationEnvironment &env)
{
  if (!EnableNMEA(env))
      return false;

    char buffer[30];
    double Pressure=pres.GetHectoPascal();
    sprintf(buffer,"POVG,C,QNH,%4.2f", Pressure);
    PortWriteNMEA(port, buffer, env);
    return true;
}

bool
JRVario::PutBugs(double bugs, OperationEnvironment &env)
{
  if (!EnableNMEA(env))
        return false;

      char buffer[30];
      sprintf(buffer,"POVG,C,BU,%4.2f", bugs);
      PortWriteNMEA(port, buffer, env);
      return true;
}

bool
JRVario::ParseNMEA(const char *String, struct NMEAInfo &info)
{
  if (!VerifyNMEAChecksum(String))
      return false;
  NMEAInputLine line(String);
  char type[16];
  line.Read(type, 16);

  if (StringIsEqual(type, "$PJRC"))
    return PJRC(line, info);

  return true;
}

static Device *
JRVarioCreateOnPort(const DeviceConfig &config, Port &com_port)
{
  return new JRVario(com_port);
}

const struct DeviceRegister jrvario_driver = {
  _T("jrvario"),
  _T("JRVario"),
  DeviceRegister::NO_TIMEOUT,
  JRVarioCreateOnPort,
};
