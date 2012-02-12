#!/bin/bash

usage_exit() {
    echo "Usage : $0 <ogv_file> [--gtest_output file] [-o|--output OUTPUT]" 1>&2
    exit 1
}
trap usage_exit ERR

OGV_FILENAME=$1
BASE_NAME=`basename $OGV_FILENAME .ogv`
BASE_DIR=`dirname $1`
MEDIA_DIR=/tmp

GETOPT=`getopt -o o: -l output:,text,gtest_output: -- "$@"` ; [ $? != 0 ] && usage_exit
eval set -- "$GETOPT"
while true
do
  case $1 in
  --gtest_output)  OUTFILE=$2      ; shift 2 ;;
  -h)   usage_exit ;;
  --)   shift ; break ;;
  *)   shift ; break ;;
  esac
done


echo "[glc_encode] ogv file name : $OGV_FILENAME"
echo "[glc_encode] file base name: $BASE_NAME"
echo "[glc_encode] file base dir : $BASE_DIR"

OUTFILE=${OUTFILE/xml:/}
if [ "${OUTFILE}" ]; then
    cat<<EOF > ${OUTFILE}
<testsuite errors="0" failures="0" name="unittest.suite.TestSuite" tests="1" time="0.0">
  <testcase classname="__main__.TestGlcEncode" name="test_glc_encode" time="0.0"></testcase>
  <system-out><![CDATA[]]></system-out>
  <system-err><![CDATA[]]></system-err>
</testsuite>
EOF
fi

MP4_FILENAME=${BASE_DIR}/${BASE_NAME}.mp4
WEBM_FILENAME=${BASE_DIR}/${BASE_NAME}.webm
PNG_FILENAME=${BASE_DIR}/${BASE_NAME}.png

echo "[glc_encode] mp4 file name : $MP4_FILENAME"
echo "[glc_encode] png file name : $PNG_FILENAME"

echo "[glc_encode] convert  from ${OGV_FILENAME} to ${MP4_FILENAME}"
arista-transcode ${OGV_FILENAME} -o ${MP4_FILENAME}

CAPTUREPOINT=`ffmpeg -i ${MP4_FILENAME} 2>&1 | grep Duration | cut -d " " -f 4 | sed  s/,// | awk 'BEGIN{FS=":"}{print ($1*60*60+$2*60+$3)/2}'`
rm -f ${PNG_FILENAME}
echo "[glc_encode] convert  from ${MP4_FILENAME} to ${PNG_FILENAME} at ${CAPTUREPOINT}"
ffmpeg -y -i ${MP4_FILENAME} -vframes 1 -ss ${CAPTUREPOINT} ${PNG_FILENAME}

exit 0
