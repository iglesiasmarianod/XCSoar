/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000 - 2009

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

#ifndef RASTERMAP_H
#define RASTERMAP_H

#include "Sizes.h"
#include <zzip/lib.h>
#include "jasper/RasterTile.h"
#include "Poco/RWLock.h"

#include <windef.h> /* for MAX_PATH */

#include "GeoPoint.hpp"

typedef struct _TERRAIN_INFO
{
  double Left;
  double Right;
  double Top;
  double Bottom;
  double StepSize;
  long Rows;
  long Columns;
} TERRAIN_INFO;


typedef struct _TERRAIN_CACHE
{
  short h;
  long index;
  unsigned int recency;
} TERRAIN_CACHE;

class RasterRounding;


class RasterMap {
 public:
  RasterMap():
    terrain_valid(false),
    max_field_value(0),
    DirectAccess(false),
    Paged(false)
    {}
  virtual ~RasterMap() {};

  inline bool isMapLoaded() {
    return terrain_valid;
  }

  short max_field_value;
  TERRAIN_INFO TerrainInfo;

  virtual void SetViewCenter(const GEOPOINT &location) {};

  bool GetMapCenter(GEOPOINT *loc);

  float GetFieldStepSize();

  // inaccurate method
  int GetEffectivePixelSize(double pixelsize);

  // accurate method
  int GetEffectivePixelSize(double *pixel_D,
                            const GEOPOINT &location);

  virtual void SetFieldRounding(const double xr, const double yr,
    RasterRounding &rounding);

  short GetField(const GEOPOINT &location,
    const RasterRounding &rounding);

  virtual bool Open(char* filename) = 0;
  virtual void Close() = 0;
  virtual void ServiceCache() {};
  virtual void ServiceFullReload(const GEOPOINT &location) {};
  bool IsDirectAccess(void) { return DirectAccess; };
  bool IsPaged(void) { return Paged; };

  // export methods to global, take care!
  virtual void LockRead();
  virtual void Unlock();

 protected:
  Poco::RWLock lock;

  bool terrain_valid;
  bool Paged;

  bool DirectAccess;

  virtual short _GetFieldAtXY(unsigned int lx,
                              unsigned int ly) = 0;
};


class RasterMapCache: public RasterMap {
 public:
  RasterMapCache() {
    terraincacheefficiency=0;
    terraincachehits = 1;
    terraincachemisses = 1;
    cachetime = 0;
    DirectAccess = false;
    if (ref_count==0) {
      fpTerrain = NULL;
    }
    ref_count++;
  }

  ~RasterMapCache() {
    ref_count--;
  }

  // shared!
  static ZZIP_FILE *fpTerrain;
  static int ref_count;

  void ServiceCache();

  virtual bool Open(char* filename);
  virtual void Close();
  virtual void LockRead();

 protected:
  TERRAIN_CACHE TerrainCache[MAXTERRAINCACHE];

  int terraincacheefficiency;
  long terraincachehits;
  long terraincachemisses;
  unsigned int cachetime;
  int SortThresold;

  short _GetFieldAtXY(unsigned int lx,
                      unsigned int ly);
  void OptimiseCache(void);
  void SetCacheTime();
  void ClearTerrainCache();
  short LookupTerrainCache(const long &SeekPos);
  short LookupTerrainCacheFile(const long &SeekPos);
  //
};


class RasterMapRaw: public RasterMap {
 public:
  RasterMapRaw():
    TerrainMem(NULL)
  {
    DirectAccess = true;
  }
  ~RasterMapRaw() {
  }
  short *TerrainMem;
  virtual void SetFieldRounding(const double xr, const double yr,
    RasterRounding &rounding);
  virtual bool Open(char* filename);
  virtual void Close();
 protected:
  virtual short _GetFieldAtXY(unsigned int lx,
                              unsigned int ly);
};


class RasterMapJPG2000: public RasterMap {
 public:
  RasterMapJPG2000();
  ~RasterMapJPG2000();

  void ReloadJPG2000(void);
  void ReloadJPG2000Full(const GEOPOINT &location);

  void SetViewCenter(const GEOPOINT &location);
  virtual void SetFieldRounding(const double xr, const double yr,
    RasterRounding &rounding);
  virtual bool Open(char* filename);
  virtual void Close();
  void ServiceFullReload(const GEOPOINT &location);

 protected:
  char jp2_filename[MAX_PATH];
  virtual short _GetFieldAtXY(unsigned int lx,
                              unsigned int ly);
  bool TriggerJPGReload;
  static int ref_count;
  RasterTileCache raster_tile_cache;
};


class RasterRounding {
public:
  RasterRounding() {};

  RasterRounding(RasterMap &map,
    const double xr, const double yr):
    DirectFine(false)
  {
    Set(map, xr, yr);
  };
  void Set(RasterMap &map,
           const double xr, 
           const double yr) 
  {
    map.SetFieldRounding(xr,yr,*this);
  }

  bool DirectFine;
  int xlleft;
  int xlltop;
  double fXrounding, fYrounding;
  double fXroundingFine, fYroundingFine;
  int Xrounding, Yrounding;
};


#endif
