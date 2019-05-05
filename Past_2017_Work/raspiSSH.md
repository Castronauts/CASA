# SSH into Raspi over USB

1. Flash Raspian image onto SD card. SD card is now labeled `boot`.

2. Open the file `config.txt` in `boot`. At the very bottom of the file on a new line add `dtoverlay=dwc2`.

3. Open the file `cmdline.txt` in `boot`. Immediately following the parameter `rootwait` add `modules-load=dwc2,g_ether`. This file uses a 'space' as a delimeter so make sure there is a single space after `rootwait` and a single space after `modules-load=dwc2,g_ether` if another item follows.

4. Create an empty file called `ssh` (no extension) to enable SSH. SSH is not enabled by default in new versions of Raspian. Easiest method is to enter `touch ssh` in the terminal while in the boot directory.

5. Power the raspi and connect your computer via USB.

6. Give the raspi time to boot and then enter `ssh pi@raspberrypi.local` into your terminal.

7. You will be prompted whether to include this new host to your list of known hosts, type `yes` and hit enter.

8. Next you will be prompted to enter the password. Default is `raspberry`.


**Note:** When SSH'ing into a different pi you will be prevented because the SSH keys do not match. Inside `~/.ssh/known_hosts` a key has been attached to `raspberrypi.local`. Since you are connecting to a different pi the keys will not match and thus you will be prevented from connecting (man-in-the-middle attack). I simply delete the SSH key associated with `raspberrypi.local` in my `known_hosts` file to get around this. There is probably a better way to fix this problem, but I find this pretty easy.
