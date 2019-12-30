# import the necessary packages and maybe more
import os
import gc
from imutils import paths
import argparse
import cv2
import numpy as np
from osgeo import gdal
import time
import easygui
import fnmatch
import time
import sys
import re
import osr
import subprocess

def striplist(l):
    return([x.strip() for x in l])

def cls():
    os.system('cls' if os.name=='nt' else 'clear')

def compLat_Long(degs, mins, secs, comp_dir):
    return (degs + (mins / 60) + (secs / 3600)) * comp_dir

def getSignOf(chifre):
    if chifre >= 0:
        return 1
    else:
        return -1

def splitxmlN(nom):
     nom=nom.split('=')
     nom=nom[0]
     return nom

# aero to map
def hrp2opk(Roll, Pitch, heading):
    Roll = np.deg2rad(Roll)
    Pitch   = np.deg2rad(Pitch)
    heading = np.deg2rad(heading)

    A_SINH = np.sin(heading)
    A_SINR = np.sin(Roll)
    A_SINP = np.sin(Pitch)

    A_COSH = np.cos(heading)
    A_COSR = np.cos(Roll)
    A_COSP = np.cos(Pitch)

    MX = np.zeros((3, 3))
    MX[0][0] =  (A_COSH *A_COSR) + (A_SINH*A_SINP*A_SINR)
    MX[0][1] =  (-A_SINH*A_COSR)+(A_COSH*A_SINP*A_SINR)
    MX[0][2] =   -A_COSP*A_SINR

    MX[1][0] = A_SINH*A_COSP
    MX[1][1] = A_COSH*A_COSP
    MX[1][2] = A_SINP


    MX[2][0] = (A_COSH*A_SINR)-(A_SINH*A_SINP*A_COSR)
    MX[2][1] = (-A_SINH*A_SINR)-(A_COSH*A_SINP*A_COSR)
    MX[2][2] =  A_COSP*A_COSR

    P = np.zeros((3, 3))
    P[0][0] = MX[0][0]
    P[0][1] = MX[1][0]
    P[0][2] = MX[2][0]
    
    P[1][0] = MX[0][1]
    P[1][1] = MX[1][1]
    P[1][2] = MX[2][1]
    
    P[2][0] = MX[2][0]
    P[2][1] = MX[1][2]
    P[2][2] = MX[2][2]

    Omega = 0
    Phi   = 0
    Kappa = 0

    Omega = np.arctan(-P[2][1]/P[2][2])
    Phi = np.arcsin(P[2][2])
    Kappa = np.arctan(-P[1][0]/P[0][0])

    Phi   = abs(np.arcsin(P[2][0]))
    Phi = Phi * getSignOf(P[2][0])
    Omega = abs(np.arccos((P[2][2] / np.cos(Phi))))
    Omega = Omega * (getSignOf(P[2][1] / P[2][2]*-1))
    Kappa = np.arccos(P[0][0] / np.cos(Phi))

    if getSignOf(P[0][0]) == getSignOf((P[1][0] / P[0][0])):
        Kappa = Kappa * -1

    Omega = np.rad2deg(Omega)
    Phi = np.rad2deg(Phi)
    Kappa = np.rad2deg(Kappa)
   
    return(Omega,Phi,Kappa)


def rotation(nx, ny, z0, omega, phi, kappa):
      #heading : ψ, roll: Φ, pitch:Θ 

      # apply yaw (around z) / yaw kappa
      xr1= nx * np.cos(kappa) - ny * np.sin(kappa)
      yr1= nx * np.sin(kappa) + ny * np.cos(kappa)
      zr1 = z0

      #apply pitch (around x) / pitch / omega
      xr2 = xr1
      yr2 = yr1 * np.cos(omega) - zr1 * np.sin(omega)
      zr2 = yr1 * np.sin(omega) + zr1 * np.cos(omega)

      # apply roll (around y) / roll / ohi
      xr3 = xr2 * np.cos(phi) - zr2 * np.sin(phi)
      yr3 = yr2
      zr3 = xr2 * np.sin(phi) + zr2 * np.cos(phi)

      return xr3, yr3, zr3

def splitxmlC(chifre):
     chifre=chifre.split('=')
     chifre=chifre[1]
     chifre = re.sub('"', '', chifre)
     chifre = float(chifre)
     return chifre

def update_progress(progress):
    barLength = 30 # Modify this to change the length of the progress bar
    
    block = int(round(barLength*progress))
    text = "\rPercent: [{0}] {1}% ".format( "#"*block + "-"*(barLength-block), int(progress*100))
    sys.stdout.write(text)
    sys.stdout.flush()

def transform_wgs84_to_utm(lon, lat, alt):    
    def get_utm_zone(longitude):
        return (int(1+(longitude+180.0)/6.0))

    def is_northern(latitude):
        """
        Determines if given latitude is a northern for UTM
        """
        if (latitude < 0.0):
            return 0
        else:
            return 1

    utm_coordinate_system = osr.SpatialReference()
    utm_coordinate_system.SetWellKnownGeogCS("WGS84") # Set geographic coordinate system to handle lat/lon  
    utm_coordinate_system.SetUTM(get_utm_zone(lon), is_northern(lat))
    
    wgs84_coordinate_system = utm_coordinate_system.CloneGeogCS() # Clone ONLY the geographic coordinate system 
    
    # create transform component
    wgs84_to_utm_transform = osr.CoordinateTransformation(wgs84_coordinate_system, utm_coordinate_system) # (<from>, <to>)
    
    return wgs84_to_utm_transform.TransformPoint(lon, lat, alt) # returns easting, northing, altitude    



dirname = easygui.diropenbox(msg=None, title="Please select a directory", default=None )
total_con=len(fnmatch.filter(os.listdir(dirname), '*.jpg'))
msg = str(total_con) +" files do you want to continue?"
title = "Please Confirm"
if easygui.ynbox(msg, title, ('Yes', 'No')): # show a Continue/Cancel dialog
    pass # user chose Continue else: # user chose Cancel
else:
    exit(0)

file_Dir = os.path.basename(dirname)

ci=0
cls()

# Attention en dur DJI 20MP 1” CMOS Sensor
Micron = 2.4107
pRot = +90

dir = os.path.join(dirname,"mosaic")
if not os.path.exists(dir):
    os.mkdir(dir)
ff = open(dirname+"\EO.txt", "w")
f = open(dir+"\list.txt", "w")

for imagePath in os.listdir(dirname):

 if fnmatch.fnmatch(imagePath.upper(), '*.JPG'):
   
   fd = open(dirname+'\\'+imagePath, 'rb')
   d= fd.read()
   imgAsString=d.decode(encoding="utf8", errors='ignore')
   #imgAsString=str(d)
   xmp_start = imgAsString.find('<x:xmpmeta')
   xmp_end = imgAsString.find('</x:xmpmeta')
   xmp_str = imgAsString[xmp_start:xmp_end+12]
   data = xmp_str.splitlines()
   data = striplist(data)
   ii=0
   fd.close()

   for i in data:
     if splitxmlN(i)=='drone-dji:Latitude':
      Latitude = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:Longitude':
      Longitude = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:AbsoluteAltitude':
      AltitudeA = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:RelativeAltitude':
      Altitude = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:GimbalRollDegree':
      Roll = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:GimbalPitchDegree':
      Pitch = splitxmlC(i)
     if splitxmlN(i)=='drone-dji:GimbalYawDegree':
      Yaw = splitxmlC(i)

   x0,y0,z0 = transform_wgs84_to_utm(Longitude,Latitude,Altitude)
   Pitch =pRot+Pitch
   Omega, Phi, Kappa = hrp2opk(Roll, Pitch, -Yaw)
  

   Omega = np.deg2rad(Omega)
   Phi = np.deg2rad(Phi)
   Kappa = np.deg2rad(Kappa)
  
   dataset = gdal.OpenEx(dirname+'\\'+imagePath, gdal.GA_ReadOnly)
   md = dataset.GetMetadata()
   xwidth=(dataset.RasterXSize)
   yheight=(dataset.RasterYSize)
 
   dataset = None

   foc = md.get('EXIF_FocalLength', None)
   foc = foc.replace("(", "")
   foc = foc.replace(")", "")


   scale = (float(Altitude)/float(foc))*1000
   GSD = (float(scale)*Micron/10000)/100
   

   ff.write(os.path.splitext(os.path.basename(imagePath))[0]+'\t'+"{:10.3f}".format(x0)+'\t'+"{:10.3f}".format(y0)+'\t' +"{:10.3f}".format(z0)+'\t'+"{:10.3f}".format(Roll)+'\t'+"{:10.3f}".format(Pitch)+'\t'+"{:10.3f}".format(Yaw)+'\n')
   ff.flush

   imgWorldE=xwidth*GSD
   imgWorldN=yheight*GSD

   # haut gauche
   x1 = (x0-(imgWorldE/2))
   y1 = (y0+(imgWorldN/2))
   # haut droite
   x2 = (x0+(imgWorldE/2))
   y2 = (y0+(imgWorldN/2))
   # bas droite
   x3 = (x0+(imgWorldE/2))
   y3 = (y0-(imgWorldN/2))
   # bas gauche
   x4 = (x0-(imgWorldE/2))
   y4 = (y0-(imgWorldN/2))
 

   # all rotations counter clockwise

   # point haut gauche
   # tranlation
   nx = x1-x0
   ny = y1-y0
   # rotation 
   xr3, yr3, zr3 = rotation(nx, ny, z0, Omega, Phi, Kappa)

   # tranlation back
   x1r = xr3 + x0
   y1r = yr3 + y0

   # point haut droite
   # tranlation
   nx = x2-x0
   ny = y2-y0
   # rotation
   xr3, yr3, zr3 = rotation(nx, ny, z0, Omega, Phi, Kappa)

   # tranlation back
   x2r = xr3 + x0
   y2r = yr3 + y0

   # point bas droite
   # tranlation
   nx = x3-x0
   ny = y3-y0
   # rotation counterclockwise
   xr3, yr3, zr3 = rotation(nx, ny, z0, Omega, Phi, Kappa)

   # tranlation back
   x3r = xr3 + x0
   y3r = yr3 + y0

   # point bas gauche
   # tranlation
   nx = x4-x0
   ny = y4-y0
   # rotation counterclockwise
   xr3, yr3, zr3 = rotation(nx, ny, z0, Omega, Phi, Kappa)

   # tranlation back
   x4r = xr3 + x0
   y4r = yr3 + y0

   ci  += 1

   # Attention code EPSG en dur
   tranlate = 'gdal_translate -of GTiff -a_srs EPSG:32648 -gcp 0 0 '+ str(x1r)+' '+ str(y1r) + \
    ' -gcp '+str(xwidth)+' 0 '+ str(x2r) +' '+ str(y2r) + \
    ' -gcp '+ str(xwidth) +' ' + str(yheight) + ' ' + str(x3r) +' '+ str(y3r) +\
    ' -gcp 0 ' + str(yheight) + ' ' + str(x4r) +' '+ str(y4r) +' '\
    + dirname+'\\'+imagePath +' '+ dir + '\\' + os.path.splitext(os.path.basename(imagePath))[0]+'_T.tif'

   warp = 'gdalwarp -co COMPRESS=NONE -tr 1 1 -s_srs EPSG:32648 -dstalpha -multi -of GTiff '\
    + dir + '\\' + os.path.splitext(os.path.basename(imagePath))[0]+'_T.tif' + ' ' + dir + \
    '\\' + os.path.splitext(os.path.basename(imagePath))[0]+'.tif'
  
   os.system('set OSGEO4W_ROOT=C:\OSGEO4~1')
   os.system('set path=%OSGEO4W_ROOT%\bin;%WINDIR%\system32;%WINDIR%;%WINDIR%\system32\WBem;C:\LAStools\bin\laszip.exe')
   os.system('SET GDAL_DATA=C:\OSGEO4~1\share\gdal')
   os.system('SET GDAL_DATA=%OSGEO4W_ROOT%\share\gdal')
   os.system('SET GDAL_DRIVER_PATH=%OSGEO4W_ROOT%\bin\gdalplugins')
   os.system('SET GDAL_DRIVER_PATH=C:\OSGEO4~1\bin\gdalplugins')
   os.system('SET GEOTIFF_CSV=C:\OSGEO4~1\share\epsg_csv')
   os.system('set JPEGMEM=100000')
   os.system('SET GDAL_DATA=%OSGEO4W_ROOT%\share\gdal')
   os.system('SET GDAL_DRIVER_PATH=%OSGEO4W_ROOT%\bin\gdalplugins')
   os.system('SET PROJ_LIB=%OSGEO4W_ROOT%\share\proj')

   print('\n')
   os.system(tranlate)
   print('\n')
   os.system(warp)
   print('\n')
   os.remove(dir + '\\' + os.path.splitext(os.path.basename(imagePath))[0]+'_T.tif')
   
   f.write(dir + '\\' + os.path.splitext(os.path.basename(imagePath))[0]+'.tif'+'\n')
   f.flush
   update_progress(ci/total_con)


f.close
f = open(dir + '\list.txt')
f.close()
ff.close()

merging = 'python C:/OSGeo4W64/bin/gdal_merge.py -of GTiff -co TFW=YES -o ' + dirname + '\merged.tif --optfile '+ dir + '\list.txt'
print('merging\n')
os.system(merging)

delforlder = 'rmdir /S /Q '+ dir
os.system(delforlder)
cls()
print('Done')