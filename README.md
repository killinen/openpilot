# OPENPILOT for MY99 BMW 540i (E39)
This project was created to solve the need for a MY99 BMW 540i (E39) to drive by itself.

## What is OPENPILOT
OPENPILOT is an open source driver assistance system that offers Automated Lane Centering and Adaptive Cruise Control for over 200 supported car makes and models. To function properly, OPENPILOT needs to be able to control the longitudinal (gas and brake) and lateral (steering) movements of the car using the CAN bus. For more information, see https://comma.ai/ and https://github.com/commaai/openpilot.

Video introducing OPENPILOT.

[![](https://i3.ytimg.com/vi/NmBfgOanCyk/maxresdefault.jpg)](https://youtu.be/NmBfgOanCyk)

OPENPILOT is quite complex robotics platform which isn't easyly explainable, nor one can quickly learn. For someone to take deeper look into how OPENPILOT works, here's couple reads: https://blog.comma.ai/openpilot-in-2021/ https://github.com/commaai/openpilot/wiki/Introduction-to-openpilot.

In short OPENPILOT is LEVEL 2 ADAS software that runs on device that has one camera for recording the drive view. Another camera for driver monitoring. The device is connected or it has integrated device that can talk to CAN bus and has high precision GPS on it (eg. Panda). OPENPILOT inputs those camera feeds runs ML model through it, evaluate car state from sensors and CAN bus messages and output from those are longitudinal and lateral control commands.

## OPENPILOT hardware
To run OPENPILOT you need hardware that can run on it. COMMA AI produces their own device COMMA THREE https://github.com/commaai/openpilot/wiki/comma-three, older HW has been also made by COMMA eg. COMMA TWO https://github.com/commaai/openpilot/wiki/comma-two and EON. You can run OPENPILOT also on linux PC.

# FrEON
FrEON is name for a phone that can run OPENPILOT, which is in 3D-printed case and have somekind of querrilla tactics cooling on it. Basically there are two phones that can run (older) OPENPILOT software, OnePlus 3T and LeEco Le Pro 3 (great name don't you think).

[![](https://i3.ytimg.com/vi/RC8wjAatwl0/maxresdefault.jpg)](https://www.youtube.com/watch?v=RC8wjAatwl0)

# Panda
Panda is device that is essential to OPENPILOT ecosystem with COMMA/FrEON device. It is CAN interfacing devices that has high precision GPS and it'll power the COMMA/FrEON (https://github.com/commaai/panda). There have been 4 different kind of Pandas, White, Grey, Black and the newest is Red. Here's a white panda (old) hardware guide: https://github.com/commaai/panda/blob/master/docs/guide.pdf.

Video showing Grey and Black Panda:
[![Video showing Grey and Black Panda](https://i3.ytimg.com/vi/0iKRq7-kywI/maxresdefault.jpg)](https://www.youtube.com/watch?v=0iKRq7-kywI)

On cars that have ADAS Pandas working principle is little diffrent where it has to intercept CAN bus messages than in my use case (I won't go any deeper here into it). In my much simpler use is just that it talk to one CAN bus and it needs to be connected to 12V, GND and IGN lines to function properly.

# Additional hardware
To OPENPILOT work correctly it needs to have lateral (steering) and longitudinal (gas and brake) control of the car. In newer cars with ADAS cababitlity this is achived by intercepting the CAN bus messages, but in older cars you usually needs to add the controlling hardware. In my case this is done for gas intercepting the gas pedal sensor signals with COMMA PEDAL (https://github.com/commaai/openpilot/wiki/comma-pedal) type of hardware and brake using my own designed BrakeModule (https://github.com/killinen/BrakeModule). I don't yet have lateral control).

Heres a quite good video that shows what is needed for making OPENPILOT work on older car. This is even older than mine, the solution are different and there is no braking capabitlity but still this show the basic idea:

[![](https://i3.ytimg.com/vi/L1u6AkSpR98/maxresdefault.jpg)](https://www.youtube.com/watch?app=desktop&v=L1u6AkSpR98&t=2s)
 
## Configuration
So devices that are interfacing with my car are:
- Panda (Grey)
- Gas interceptor
- BrakeModule

These all talk to each other vai BMW's original CAN bus:

![Bus Topology I -K-M-P-Can-Diagnostic](https://user-images.githubusercontent.com/37126045/205412230-d4519533-0346-42c4-b3b5-591e453f1f85.jpg)

On the same CAN bus are as shown in the picture are:
- Panda (OP)
- BrakeModule (BM)
- Steering Angle Sensor (LEW)
- BOSCH ABS module (DSCIII)
- Motor contor unit (DME)

Communication between Panda and FrEON takes place via USB.

## My forks software mods in default branch:
Few mods have been done to this fork order to work with my car and my likenings. On high level:
- Add E39 CAN msgs configuration to opendbc
- Detect BMW as OLD_CAR which is detected as TOYOTA COROLLA (fingerprinting)
- Panda SW mods to accept BMW CAN msg's
- Disable steering requests
- Enable vision only longitudinal control and improve it (ACC wo radar)
- Made stop and go work good enough
- Made smoothing of gas output to lessen jerk when accelerating
- Add OP3T support

Default branch is based on openpilot version 0.8.2, in December of 2022 there is already openpilot version 0.9 which is much more capable than this older one. Unfortunetally my hardware isn't supported anymore.

To be clear my car does not yet have steering capability but this system is used as vision based ACC which have stop and go functionality. Even this emproves my driving experience to the point that if for some reason I don't have OPENPILOT working it bums me.

### Below is notes from Shane Smiskol whos Stock Additions fork is integrated in my fork, which has great additions to stock OPENPILOT

# Stock Additions [Update 3](/SA_RELEASES.md) (0.8.2)

Stock Additions is a fork of openpilot designed to be minimal in design while boasting various feature additions and behavior improvements over stock. I have a 2017 Toyota Corolla with comma pedal, so most of my changes are designed to improve the longitudinal performance.

Want to request a feature or create a bug report? [Open an issue here!](https://github.com/ShaneSmiskol/openpilot/issues/new/choose) Want to reach me to debug an issue or have a question? `Shane#6175` on Discord!

[View Stock Additions Changelog](/SA_RELEASES.md)

---
## Highlight Features

### Behavior Changes
* [**Dynamic follow (now with profiles!)**](#dynamic-follow-3-profiles) - 3 + auto profiles to control distance
  * [**`auto-df` model for automatic distance profile switching**](#Automatic-DF-profile-switching)
* **Lane Speed**  [***❗ALL LANE SPEED FEATURES REMOVED TEMPORARILY❗***](https://github.com/ShaneSmiskol/openpilot/blob/stock_additions/SA_RELEASES.md#stock-additions-v066---2021-02-27-082)
  * [**Lane Speed Alerts**](#Lane-Speed-alerts) - alerts for when an adjacent lane is faster
  * [**Dynamic camera offsetting**](#Dynamic-camera-offset-based-on-oncoming-traffic) - moves you over if adjacent lane has oncoming traffic
* [**Dynamic gas**](#dynamic-gas) - smoother gas control
* [**Adding derivative to PI for better control**](#pi---pid-controller-for-long-and-lat) - lat: smoother control in turns; long: fix for comma pedal overshoot

### General Features
* [**NEW❗ Smoother long control using delay**](#new-compensate-for-longitudinal-delay-for-earlier-braking) - using an accel delay, just like for lateral
* [**Customize this fork**](#Customize-this-fork-opEdit) - easily edit fork parameters with support for live tuning
* [**Automatic updates**](#Automatic-updates)
* [**ZSS Support**](#ZSS-support) - takes advantage of your high-precision Zorrobyte Steering Sensor
* [**Offline crash logging**](#Offline-crash-logging) - check out `/data/community/crashes`

### Visual Changes (LINKS WIP)
* [**Colored the lane lines**]() - based on distance from car
* [**Colored model path**]() - based on curvature

## Documentation
* [**Quick Installation**](#Quick-installation)
* [**Branches**](#Branches)
* [**Videos**](#Videos)

---
## Behavior changes

### Dynamic follow (3 profiles)
Dynamic follow aims to provide the stock (Toyota) experience of having three different distance settings. Dynamic follow works by dynamically changing the distance in seconds which is sent to the long MPC to predict a speed to travel at. Basically, if the lead is decelerating or might soon, increase distance to prepare. And if the lead is accelerating, reduce distance to get up to speed quicker.

Dynamic follow works if openpilot can control your vehicle's gas and brakes (longitudinal). [Check if openpilot can control your vehicle's longitudinal from this list.](https://github.com/commaai/openpilot#supported-cars)

Just use the button on the button right of the screen while driving to change between these profiles:
  * [`traffic`](#Videos) - Meant to keep you a bit closer in traffic, hopefully reducing cut-ins. Always be alert, as you are with any driving assistance software.
  * `relaxed` - This is the default dynamic follow profile for casual driving.
  * `stock` - This is the stock 1.8 second profile default in stock openpilot, with no dynamic follow mods. The previous roadtrip profile was closer than a *true road trip* profile, this is more in line with that intention.
  * [`auto`](#Automatic-DF-profile-switching) - The auto dynamic follow model was trained on about an hour of me manually cycling through the different profiles based on driving conditions, this profile tries to replicate those decisions entirely on its own.

<p align="center">
  <img src=".media/df_profiles.jpg?raw=true">
</p>

---
### Automatic DF profile switching
I've trained a custom model with Keras that takes in the past 35 seconds of your speed, the lead's speed and the lead's distance. With these inputs, it tries to correctly predict which profile is the best for your current situation.

It's only been trained on about an hour of data, so it's not perfect yet, but it's great for users who just want to set it and forget it. **To enable the `auto` profile, simply tap the profile changing button for dynamic follow until it reaches the `auto` profile!**

If you're annoyed by the silent alerts that show when the model has changed the profile automatically, just use [opEdit](#Customize-this-fork-opEdit) and set `hide_auto_df_alerts` to `True`. Auto profile and model will remain functional but will not show alerts.

Resources:
- [The auto-df repo.](https://github.com/ShaneSmiskol/auto-df)
- [The model file.](https://github.com/ShaneSmiskol/openpilot/blob/stock_additions/selfdrive/controls/lib/dynamic_follow/auto_df.py)
- I converted the Keras model to be able to run with pure NumPy using [Konverter](https://github.com/ShaneSmiskol/Konverter).

---
### Lane Speed alerts
This feature alerts you of faster-travelling adjacent lanes and can be configured using the on-screen *LS* button on the bottom right to either be disabled, audible, or silent.

The idea behind this feature is since we often become very relaxed behind the wheel when being driven by openpilot, we don't always notice when we've become stuck behind a slower-moving vehicle. When either the left or right adjacent lane is moving faster than your current lane, LaneSpeed alerts the user that a faster lane is available so that they can make a lane change, overtaking the slower current lane. Thus saving time in the long run on long road trips or in general highway driving!

The original idea is thanks to [Greengree#5537](https://github.com/greengree) on Discord. This feature is available at 35 mph and up.

---
### Dynamic camera offset (based on oncoming traffic)
This feature automatically adjusts your position in the lane if an adjacent lane has oncoming traffic. For example, if you're on a two-lane highway and the left adjacent lane has oncoming cars, LaneSpeed recognizes those cars and applies an offset to your `CAMERA_OFFSET` to move you over in the lane, keeping you farther from oncoming cars.

**This feature is available from 35 to ~60 mph due to a limitation with the Toyota radar**. It may not recognize oncoming traffic above 60 mph or so. To enable or disable this feature, use `opEdit` and change this parameter: `dynamic_camera_offset`.

---
### Dynamic gas
Dynamic gas aims to provide a smoother driving experience in stop and go traffic (under 20 mph) by reducing the maximum gas that can be applied based on your current velocity, the relative velocity of the lead, the acceleration of the lead, and the distance of the lead. This usually results in quicker and smoother acceleration from a standstill without the jerking you get in stock openpilot with comma pedal (ex. taking off from a traffic light). It tries to coast if the lead is just inching up, it doesn’t use maximum gas as soon as the lead inches forward. When you are above 20 mph, relative velocity and the current following distance in seconds is taken into consideration.

All cars that have a comma pedal are supported! However to get the smoothest acceleration, I've custom tuned gas curve profiles for the following cars:

pedal cars:
  * 2017 Toyota Corolla (non-TSS2)
  * Toyota RAV4 (non-TSS2)
  * 2017 Honda Civic
  * 2019 Honda Pilot

non-pedal cars:
  * None yet

If you have a car without a pedal, or you do have one but I haven't created a profile for you yet, please let me know and we can develop one for your car to test.

---
### PI -> PID Controller for Long and Lat
(long: longitudinal, speed control. lat: latitudinal, steering control)

**Changes for lat control: (NEW❗)**
- Adding the derivative componenet to lat control greatly improves the turning performance of openpilot, I've found it loses control much less frequently in both slight and sharp curves and smooths out steering in all situations. Basically it ramps down torque as your wheel approaches the desired angle, and ramps up torque quicky when your wheel is moving away from desired.

  ***Currently Supported Cars: (when param `use_lqr` is False)***
  - 2017 Toyota Corolla
  - TSS2 Toyota Corolla (when param `corollaTSS2_use_indi` is False) - tune from birdman!
  - All Prius years (when param `prius_use_pid` is True) - tune from [Trae](https://github.com/d412k5t412)!

**Changes for long control:**
- I've added a custom implementation of derivative to the PI loop controlling the gas and brake output sent to your car. Derivative (change in error) is calculated based on the current and last error and added to the class's integral variable. It's essentially winding down integral according to derivative. It helps fix overshoot on some cars with the comma pedal and increases responsiveness (like when going up and down hills) on all other cars! Still need to figure out the tuning, right now it's using the same derivative gain for all cars. Test it out and let me know what you think!

  Long derivative is disabled by default due to only one tune for all cars, but can be enabled by using [opEdit](#Customize-this-fork-opEdit) and setting the `enable_long_derivative` parameter to `True`. It works well on my '17 Corolla with pedal.

---
## General Features

### NEW❗ Compensate for longitudinal delay for earlier braking
This just simply uses desired future acceleration for feedforward rather than current desired acceleration. openpilot already compensates for steering delay, but not longitudinal. This adds that, replacing the previous ***experimental*** feature called eager accel which tried to fix the same issues; jerky and late braking. Now we more correctly compensate for delay.

By default, we assume a 0.4 second delay from sending acceleration to seeing it realized, which is tunable with the opEdit param `long_accel_delay`. Raise if braking too late, lower if braking too early. Stock openpilot is 0.0 (no delay).

---
### Customize this fork (opEdit)
This is a handy tool to change your `opParams` parameters without diving into any json files or code. You can specify parameters to be used in any fork's operation that supports `opParams`. First, ssh in to your EON and make sure you're in `/data/openpilot`, then start `opEdit`:
```python
cd /data/openpilot
python op_edit.py  # or ./op_edit.py
```

[**To see what features opEdit has, click me!**](/OPEDIT_FEATURES.md)

🆕 All params now update live while driving, and params that are marked with `static` need a reboot of the device, or ignition.

Here are the main parameters you can change with this fork:
- **Tuning params**:
  - `camera_offset` **`(live!)`**: Your camera offset to use in lane_planner.py. Helps fix lane hugging
  - `steer_ratio` **`(live!)`**: The steering ratio you want to use with openpilot. If you enter None, it will use the learned steer ratio from openpilot instead
  - [`enable_long_derivative`](#pi---pid-controller-for-long-and-lat): This enables derivative-based integral wind-down to help overshooting within the PID loop. Useful for Toyotas with pedals or cars with bad long tuning
  - [`use_lqr`](#pi---pid-controller-for-long-and-lat): Enable this to use LQR for lateral control with any car. It uses the RAV4 tuning, but has proven to work well for many cars
- **General fork params**:
  - `alca_no_nudge_speed`: Above this speed (mph), lane changes initiate IMMEDIATELY after turning on the blinker. Behavior is stock under this speed (waits for torque)
  - `upload_on_hotspot`: Controls whether your EON will upload driving data on your phone's hotspot
  - [`update_behavior`](#Automatic-updates): `off` will never update, `alert` shows an alert on-screen. `auto` will reboot the device when an update is seen
  - `disengage_on_gas`: Whether you want openpilot to disengage on gas input or not
- **Dynamic params**:
  - `dynamic_gas`: Whether to use [dynamic gas](#dynamic-gas) if your car is supported
  - `global_df_mod` **`(live!)`**: The multiplier for the current distance used by dynamic follow. The range is limited from 0.85 to 2.5. Smaller values will get you closer, larger will get you farther. This is applied to ALL profiles!
  - `min_TR` **`(live!)`**: The minimum allowed following distance in seconds. Default is 0.9 seconds, the range of this mod is limited from 0.85 to 1.3 seconds. This is applied to ALL profiles!
  - `hide_auto_df_alerts`: Hides the alert that shows what profile the model has chosen
  - [`dynamic_camera_offset`](#Dynamic-camera-offset-based-on-oncoming-traffic): Whether to automatically keep away from oncoming traffic. Works from 35 to ~60 mph
    - [`dynamic_camera_offset_time`](#Dynamic-camera-offset-based-on-oncoming-traffic): How long to keep the offset after losing the oncoming lane/radar track in seconds
  - `dynamic_follow`: *Deprecated, use the on-screen button to change profiles*
- **Experimental params**:
  - `support_white_panda`: This allows users with the original white panda to use openpilot above 0.7.7. The high precision localizer's performance may be reduced due to a lack of GPS
  - [`prius_use_pid`](#pi---pid-controller-for-long-and-lat): This enables the PID lateral controller with new a experimental derivative tune
  - `corollaTSS2_use_indi`: Enable this to use INDI for lat with your TSS2 Corolla *(can be enabled for all years by request)*
  - `standstill_hack`: Some cars support stop and go, you just need to enable this

A full list of parameters that you can modify are [located here](common/op_params.py#L40).

An archive of opParams [lives here.](https://github.com/ShaneSmiskol/op_params)

Parameters are stored at `/data/op_params.json`

---
### opEdit Demo
<img src=".media/op_edit.gif?raw=true" width="1000">

---
### Automatic updates
When a new update is available on GitHub for Stock Additions, your EON/C2 will pull and reset your local branch to the remote. It then queues a reboot to occur when the following is true:
- your EON has been inactive or offroad for more than 5 minutes
- `update_behavior` param is set to `auto`

Therefore, if your device sees an update while you're driving it will reboot approximately 5 to 10 minutes after you finish your drive, it resets the timer if you start driving again before the time is up.

---
### ZSS Support
If you have a Prius with a ZSS ([Zorrobyte](https://github.com/zorrobyte) Steer Sensor), you can use this fork to take full advantage of your high-precision angle sensor! Added support for ZSS with [PR #198](https://github.com/ShaneSmiskol/openpilot/pull/198), there's nothing you need to do. Special thanks to [Trae](https://github.com/d412k5t412) for helping testing the addition!

If you have a ZSS but not a Prius, let me know and I can add support for your car.

---
### Offline crash logging
If you experience a crash or exception while driving with this fork, and you're not on internet for the error to be uploaded to Sentry, you should be able to check out the directory `/data/community/crashes` to see any and all logs of exceptions caught in openpilot. Simply view the logs with `ls -lah` and then `cat` the file you wish to view by date. This does not catch all errors, for example scons compilation errors or some Python syntax errors will not be caught, `tmux a` is usually best to view these (if openpilot didn't start).

❗***Quickly view the latest crash:*** `cat /data/community/crashes/latest.log`

Feel free to reach out to me on [Discord](#stock-additions-v066-082) if you're having any issues with the fork!

---
## Documentation

### Quick Installation
To install Stock Additions, just run the following on your EON/C2 (make sure to press enter after each line):

```
cd /data/
mv openpilot openpilot.old  # or equivalent
git clone -b stock_additions --single-branch https://github.com/shanesmiskol/openpilot --depth 1
reboot
```

The `--depth 1` flag shallow clones the fork, it ends up being about 90 Mb so you can get the fork up and running quickly. Once you install Stock Additions, [automatic updating](#Automatic-updates) should always keep openpilot up to date with the latest from my fork!

*Or use the [emu CLI](https://github.com/emu-sh/.oh-my-comma) to easily switch to this fork's default branch: `emu fork switch ShaneSmiskol`. The initial setup may take longer than the above command, but you gain the ability to switch to any fork you want.*

*Or (last or, I promise!) you can use my handy fork installation link during NEOS setup after a factory reset: **https://smiskol.com/fork/shane***

---
### Branches
Most of the branches on this fork are development branches I use as various openpilot tests. The few that more permanent are the following:
  * [`stock_additions`](https://github.com/ShaneSmiskol/openpilot/tree/stock_additions): This is similar to stock openpilot's release branch. Will receive occasional and tested updates to Stock Additions.
  * `stock_additions-devel` or `SA-staging`: My development branch of Stock Additions I use to test new features or changes; similar to the master branch. Not recommendeded as a daily driver.

---
### Archive Stock Additions branches
* [Stock Additions 0.7](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-07)
* [Stock Additions 0.7.1](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-071)
* [Stock Additions 0.7.4](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-074)
* [Stock Additions 0.7.5](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-075)
* [Stock Additions 0.7.7](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-077)
* [Stock Additions 0.7.10](https://github.com/ShaneSmiskol/openpilot-archive/tree/stock_additions-0710)
* [Stock Additions 0.8](https://github.com/ShaneSmiskol/openpilot/tree/stock_additions-08)

---
### Videos
Here's a short video showing how close the traffic profile was in `0.7.4`. In `0.7.5`, the traffic profile is an average of 7.371 feet closer from 18 mph to 90 mph. Video thanks to [@rolo01](https://github.com/rolo01)!

[![](https://img.youtube.com/vi/sGsODeP_G_c/0.jpg)](https://www.youtube.com/watch?v=sGsODeP_G_c)

---
If you'd like to support my development of Stock Additions with a [dollar for a RaceTrac ICEE](https://paypal.me/ssmiskol) (my PayPal link). Thanks! 🥰
