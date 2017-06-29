# Corx

Corx is proof-of-concept software for a low-cost correlated antenna array using inexpensive RTL-SDR dongles and single-board computers. More information about the project can be obtained at [antennaarray.co.za](http://antennaarray.co.za/).


## Installation
### Tiles
See [odroid.md](odroid.md).

### Server
Install prerequisites (Ubuntu 16.04):

```
sudo apt install python-numpy python-matplotlib
pip install --user pyftpdlib
```


## Quick start: many tiles controlled from central server

Configure all the necessary flags and settings in `corx/experiments/flags.cfg` and `corx/experiments/settings.sh`.


### Odroids

On each Odroid:

```
cd ~/corx/experiments
./multicorx_server.sh --hostid=A  # replace 'A' with a unique identifier
```
     
Multicorx writes all output from the `corx_rx` subprocesses and the EXEC commands to a log file. It might be useful for debugging to tail the newest multicorx log:

    ls -t log/multicorx*.log | head -1 | xargs tail -f


### Server

If necessary, configure firewall to allow inbound port 2121 (TCP), e.g.

    sudo iptables -I INPUT -p tcp --dport 2121 -j ACCEPT


Start the necessary services on the server:
```bash
screen
cd corx/experiments/
./ftp_upload_server.sh

# New screen window (Ctrl-a c)
cd corx/experiments/correlate_server
./correlate_monitor.sh

# New screen window (Ctrl-a c)
cd corx/experiments
./puppeteer.sh
```

Furthermore, it might be useful to monitor the uploads dir, e.g.

    watch ls -lh /tmp/uploads/

You can send commands to the tiles with `multicorx_remote.sh`, e.g.

    ./multicorx_remote.sh "exit"
       
to terminate all receiver instances, or

    ./multicorx_remote.sh "exec sudo systemctl poweroff"

to shut down the tiles.


## Overview of modules

### Corx (single RTL-SDR capture instance)

At the core of the receiver software is `corx_rx`, the software that tracks the carrier signal, detects the synchronisation pulses from the beacons and captures data to `.corx` files. Refer to [odroid.md](odroid.md) for build instructions.

`experiments/corx_rx.sh` is a wrapper around `corx_rx`. It injects the settings and flags provided in `experiments/flags.cfg` and `experiments/settings.sh`.

Examples (assuming that `experiments/` is the working directory``):

 - Capture from RTL #0 and quit:
   
       ./run_rx --device_index=0

 - Interactive prompt for controlling the receiver:
      
       ./run_rx --interactive


Interactive commands:

 - `help`: Show a list of valid commands.
 - `stop`: Switch to stop mode. No data is being sampled from the SDR device.
 - `standby`: Switch to standby mode. Data will be sampled from the SDR device to keep it warm, but will be discarded immediately.
 - `lock`: Lock to the carrier, but do not capture ant data.
 - `capture`: Lock the carrier, search for synchronisation pulses, capture data to the `.corx` file as well as noise (data with the preamp switched off). Will switch back to the last inactive mode (`stop` or `standby`) when finished or on failure.
 - `output`: Set the file to which the captured data should be written, e.g. `output data.corx`. The file will be overwritten once a new capture session starts, so be sure to change the output file before issuing the `capture` command. The output file can only be changed when the receiver is in an active mode (`stop` or `standby`).
 - `wait`: Wait for the capture session to complete before executing the next command, i.e. wait for the receiver to switch back to an inactive mode.
 - `set`: Set new flag values (e.g. `set --slice=0-100` or `set --capture_time=30`). Be careful when using this command. Any invalid flag or syntax will terminate the program. This command may only be used when the receiver is in the `stop` mode.
 - `exit`: Stop the receiver and terminate the program.


### Multicorx

When multiple RTL-SDR devices are connected to a single host, each RTL-SDR needs to be controlled by a separate `corx_rx` instance. Multiple `corx_rx` instances on a single host can be controlled simultaneously using `src/multicorx.py`.

Examples (`experiments/` is the working directory):

 - Interactive prompt for controlling four receivers simultaneously:

       ../src/multicorx.py --num=4

 - Four receivers controlled via a local socket:
 
       ../src/multicorx.py --num=4 --socket --host=127.0.0.1

 - Four receivers controlled from another computer via a network socket with arbitrary shell command execution enabled:
 
       ../src/multicorx.py --num=4 --socket --host=0.0.0.0 --hostid=A --allow-exec
       
   Remember to specify a unique `hostid` for each multicorx host.


Multicorx interactive commands:

 - Corx commands (help, stop, standby, lock, capture, output, wait, set, exit) will be forwarded to all corx instances. Any occurence of `{hostid}` will be replaced by the string specified by the `--hostid` flag and any occurrence of `{rxid}` will be replaced by the local index of the corx instance.
 - `exec`: execute an arbitrary shell command (requires `--allow-exec` flag to be set)
 - `exec_when_done` execute an arbitrary shell command when all the corx instances on the multicorx host are idle (i.e. when done capturing).

The script `multicorx_server.sh` is a wrapper for `multicorx.py` that will use the settings in `settings.sh`, create a RAM drive for temporary storage of the captured data and start multicorx with four corx instances and a socket server. Remember to specify a unique hostid, e.g. `./multicorx_server.sh --hostid=A`.

       
### Multicorx remote
The `multicorx_remote.py` script is used to control a `multicorx.py` instance listening on a socket. It can be used to control a local multicorx instance or to control multiple correlators on multiple hosts in parallel.

In addition to the multicorx commands specified above, `multicorx_remote.py` also accepts the following commands:

 - `wait`: wait for all corx instances on all multicorx hosts to become idle (i.e. finish data acquisition).
 - `wait_exec`: wait for all shell commands executed through the `exec` and `exec_when_done` commands to finish execution.

Examples (`experiments/ is the working directory`):

 - Connect to multicorx running on localhost, read commands from interactive prompt
   until SIGHUP:

       ../src/multicorx_remote.py

 - Connect to multicorx running on localhost, run the given commands and exit:

       ../src/multicorx_remote.py standby capture

 - Connect to two remote multicorx hosts, read commands from interactive prompt
   (one using the default port, another with a custom port):

       ../src/multicorx_remote.py -r 192.168.0.100,192.168.0.101:1234

 - Command three multicorx hosts to capture data and wait until completion:

       ../src/multicorx_remote.py --remote=odroid1,odroid2,odroid3 capture wait

 - Command three remote multicorx hosts to terminate:

       ../src/multicorx_remote.py --remote=odroid1,odroid2,odroid3 exit

 - Execute an arbitrary command on the remote hosts:

       ../src/multicorx_remote.py --remote=odroid1,odroid2,odroid3 "exec ./noise.sh init"

 - Command three multicorx hosts to set the output file, capture data to the specified output files, execute a shell command on each host directly after data acquisition that will upload the data, wait for all hosts to finish data acquisition as well as to finish executing the shell command (i.e. finish uploading the data), and delete the output file from the multicorx hosts:

       ../src/multicorx_remote --remote=odroid1,odroid2,odroid3 \
           "output out/data_{hostid}{rxid}.corx" \
           "capture" \
           "exec_when_idle ./ftp_upload_client.sh 192.168.2.1 out/*.corx" \
           "wait" \
           "wait_exec" \
           "exec rm out/*.corx"


In addition to the command-line interface, `multicorx_remote.py` also provides a Python API to simplify scripting. See `puppeteer.py` as an example of using the Python API.

The script `multicorx_remote.sh` is a wrapper for `multicorx_remote.py` that will connect to the receivers specified in `settings.sh`.


### Correlator

`src/correlate.py` calculates correlation coefficients from data in two .corx files. Examples:

 - Calculate and plot auto-correlation and cross-correlation between `rxA0.corx` and `rxA1.corx`:

       python correlate.py rxA0.corx rxA1.corx --plot

 - Calculate correlation between `rxA0.corx` and `rxA1.corx` and store results in `corr.npz`:

       python correlate.py rxA0.corx rxA1.corx -o corr.npz

 - Plot results stored in `corr.npz`:

       python correlate.py --plot corr.npz
    

### Correlate server
The correlate server, `correlate_server.py`, correlates all combinations of groups of incoming .corx files using multiple parallel correlators. The path to new corx files are continously read from standard input. The script `correlate_monitor.sh` is a wrapper for `correlate_server.py` that will use inotifywait to monitor a directory for new files and write the path of the corx files to `correlate_server.py` as they arrive, effectively correlating incoming `.corx` files as they arrive.


### Temporary FTP server and client
Refer to `ftp_upload_server.sh` and `ftp_upload_client.sh`.


### Puppeteer (script controlling the system)
`puppeteer.py` is the master script that controls a system of multiple receivers spread over multiple hosts as a whole.
`puppeteer.sh` is a shim that executes `puppeteer.py` with the settings specified in `settings.sh`.
