CORX_PATH="temp/"
CORRS_PATH="out/"
LOG_PATH="log/"
PARAMS="--flagfile=flags.cfg --wisdom=$HOME/corx.wisdom --input=rtlsdr"
RAMDISK_SIZE=900M
CAPTURE_NOISE_OFF_REPEAT=1

NOISE_GPIO=(
  238  # Pin 12
  246  # Pin 16
  233  # Pin 18
  231  # Pin 22
)
