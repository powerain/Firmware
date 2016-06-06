#!/bin/bash
FILENAME="$1"
TIMEFILE="/tmp/loopfile.out" > $TIMEFILE
SCRIPT=$(basename $0)
SA=""

function usage()
{
  echo -e "\nUSAGE: $SCRIPT file \n"
  exit 1
}

function while_read_bottm()
{
while read LINE
do
  temp=`echo $LINE |  awk '{print  $15 $16 $17 $18 $19 $20 $21 $22 $23 $24 $25}'`
#temp=`echo $LINE`
  if [ -n "$temp" ]; then
    SA="$temp"
  else
    LINE=${LINE}" "${SA}
  fi
  echo $LINE
  
done < $FILENAME
}

#处理GPS数据：
#1.把GPS数据附加到imu数据之后，合并为一行
#2.把没有gps数据的行复印补齐
function deal_gps()
{
SA=""
L0=""
L1=""
while read LINE
do
  #echo $LINE

  temp=`echo $LINE |  awk '{print  $1 $2}'`
  #temp=`awk '{print  $1 }' $LINE`
  if [ "$temp"x = "MSGIMU:"x ]; then
    read L1
    temp1=`echo $L1 |  awk '{print  $1 $2}'`
    if [ "$temp1"x = "MSGGPS:"x ]; then
      SA="$L1"
      echo "$LINE"" ""$SA" >> "$FILENAME.data.03"
    else
      echo "$LINE"" ""$SA" >> "$FILENAME.data.03"
      echo "$L1"" ""$SA" >> "$FILENAME.data.03"
    fi
  elif [ "$temp"x = "MSGGPS:"x ]; then
    SA="$LINE"
  fi

  #echo $temp
#  temp=`echo $LINE |  awk '{print  $15 $16 $17 $18 $19 $20 $21 $22 $23 $24 $25}'`
#temp=`echo $LINE`
#  if [ -n "$temp" ]; then
#    SA="$temp"
#  else
#    LINE=${LINE}" "${SA}
#  fi
#  echo $LINE
  
done < "$FILENAME.data.02"
}

if [ $# -lt 1 ] ; then
  usage
fi

if [ -f "$FILENAME.data.00" ]; then
 rm "$FILENAME.data.00"
fi

if [ -f "$FILENAME.data.01" ]; then
 rm "$FILENAME.data.01"
fi

if [ -f "$FILENAME.data.02" ]; then
 rm "$FILENAME.data.02"
fi

if [ -f "$FILENAME.data.03" ]; then
 rm "$FILENAME.data.03"
fi

if [ -f "$FILENAME.data" ]; then
 rm "$FILENAME.data"
fi

if [ -f "$FILENAME.IMU" ]; then
 rm "$FILENAME.IMU"
fi

if [ -f "$FILENAME.GPS" ]; then
 rm "$FILENAME.GPS"
fi

./sdlog2_dump.py $FILENAME -v > $FILENAME.data.00

#删掉前2行数据（格式数据）
sed 1,2d "$FILENAME.data.00" > "$FILENAME.data.01"
fgrep "MSG IMU" "$FILENAME.data.01" > $FILENAME.IMU
fgrep "MSG GPS" "$FILENAME.data.01" > $FILENAME.GPS
 
#行末加上一个逗号
sed 's/$/&,/g' "$FILENAME.data.01" > "$FILENAME.data.02"

deal_gps
#while_read_bottm 

sed 's/MSG IMU://g;s/AccX=//g;s/AccY=//g;s/AccZ=//g;s/GyrX=//g;s/GyrY=//g;s/GyrZ=//g;s/MagX=//g;s/MagY=//g;s/MagZ=//g;s/tA=//g;s/tG=//g;s/tM=//g;s/MSG GPS: TS=//g;s/GPST=//g;s/Lat=//g;s/Lon=//g;s/Alt=//g;s/Vn=//g;s/Ve=//g;s/Vd=//g;s/nSat=//g'  "$FILENAME.data.03" > "$FILENAME.data"

