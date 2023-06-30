*** Time synchronization between machines

It appears that the jetsons on board do not have some power storage device to keep their
RTC alive when the robot is powered down. As such, the computers have different / wrong
times that needs to be fixed every boot. chrony is used to perform synchronization. It
conflicts with =systemd-timesyncd= service. On all computers, run:
#+begin_src shell
  systemctl disable --now systemd-timesyncd.service
  sudo apt install chrony
#+end_src

On unitree, modify =/etc/chrony/chrony.conf= file to add (right after all pools):
#+begin_example
# Enable this computer to be a valid NTP server for local network
local stratum 10
allow 192.168.123.0/24
#+end_example

Then run:
#+begin_src shell
  systemctl restart chrony.service
#+end_src

This enables the use of unitree machine as an NTP server for jetsons.

On each jetson, modify =/etc/chrony/chrony.conf= file, comment out all pool links, then
add (again, right after pools):
#+begin_example
server 192.168.123.1 minpoll -1 maxpoll -1
#+end_example
The `minpoll` and `maxpoll` options determine the minimum and maximum polling rate to negotiate the time with the server. The value is powers of two so -1 means 0.5 seconds. 
Then run
#+begin_src shell
  systemctl restart chrony.service
#+end_src

Verify using =timedatectl= that the time stamps are synchronized between different
machines.

*NOTE*: What we really care is that the computers on Unitree are synchronized between
each other, even if they may be slightly off globally. However, global synchronization
becomes important when we need to perform package updates