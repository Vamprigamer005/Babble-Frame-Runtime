<p align="center"><img src="runtime/resources/icons/bfr_logo_128.png" alt="Babble Frame Runtime" width="128"></p>

# Babble Frame Runtime

Eye and mouth expression tracking that runs on the Steam Frame itself.

> **Fork:** adds the HTC Vive Facial Tracker as a mouth camera; see [About this fork](#about-this-fork-vive-facial-tracker-support) at the end.

Babble Frame Runtime reads the Steam Frame's eye cameras, and a wired Babble face tracker for the mouth, and runs a set of optimized neural networks to output expression values (eye openness, widening, pupil dilation; jaw, lips, cheeks and
tongue), which it sends to VRCFaceTracking on your PC.

It is installed as a SteamVR driver for settings integration. The driver starts the runtime as a separate process and restarts it if it stops,
so a problem in the runtime never affects SteamVR.

## Features

- **Eye expressions:** openness, widen and pupil dilation per eye.
- **Mouth tracking:** 43 blendshapes from a wired Babble face tracker plugged into the headset.
- **Gaze:** each eye's own gaze from the headset's eye service, or SteamVR's combined gaze.
- **Presence:** tracking pauses while the headset is off the head (proximity sensor), and the cameras turn off.
- **SteamVR Settings:** an Expression Tracking section to turn mouth and eye tracking on and off and set the eye
  tracking rate.
- **Outputs:**
  - OSC in the Baballonia format for VRCFaceTracking.
  - A plugin interface for other outputs.
- **Dataset capture:** an optional guided recording app, installed from the overlay's Diagnostics page. 

Work in progress. The mouth and eye models are prototypes; the runtime keeps models swappable so better ones can be
dropped in.

## Install

On the Steam Frame, switch to Desktop Mode, open Konsole and paste:

```
curl -fsSL https://raw.githubusercontent.com/Project-Babble/Babble-Frame-Runtime/main/tools/get.sh | bash
```

It downloads the newest release, checks it against the release's SHA-256 checksum, installs it in your home folder and restarts SteamVR. Its settings are in SteamVR Settings, under Expression Tracking. The Babble tab in the SteamVR dashboard is on after you install, and updates appear there next to the version. To hide the tab, turn off Show App Window.
To uninstall, run `~/.local/share/babble-frame-runtime/uninstall.sh`.

## VRCFT Module

Until further notice, this [VRCFT Module](https://cdn.discordapp.com/attachments/1279163592440283240/1561139947597992016/VRCFaceTracking.Baballonia.zip?ex=6ade7ee0&is=6add2d60&hm=92b516967feec073c70adf231a947d09ca4caf820d78a6a73157607361b80b13&) is required 

## Building from source

1. Get the source on a PC and copy it to the headset.
2. Build ExecuTorch once, then build the driver, on the headset.
3. Register the driver with `vrpathreg.sh`, then restart SteamVR.


## Build profiles

`-DBABBLE_PROFILE=release` (what releases ship) builds the USB Babble tracker as the only mouth; it leaves out the
Frame's front-port mouth cameras and their camera arbiter, the visualizer launcher, the camera-stream switch and the
unused models. Every profile has all three outputs: VRCFaceTracking, VRChat Native and Babble OSC. The default, `dev`, builds everything including unfinished features. 
## License

Licensed under the [GNU General Public License v3.0](LICENSE). Third-party code in `third_party/` keeps its own license.

## About this fork: Vive Facial Tracker support

This fork of [Project-Babble/Babble-Frame-Runtime](https://github.com/Project-Babble/Babble-Frame-Runtime) lets an
**HTC Vive Facial Tracker** plugged into the Steam Frame's USB-C port be the mouth camera, so no PC cable is needed.
Everything else is the upstream runtime.

The tracker is a UVC camera that stays dark until its sensor is set up and its stream is turned on through a vendor
Extension Unit. The runtime now sends that sequence (from Project Babble's
[reverse engineering of SRanipal](https://docs.babble.diy/blog/reverse-engineering-the-vive-facial-tracker) and the
Babble App's `vivefacialtracker` module, MIT, DragonDreams GmbH 2024), streams YUY2 400x400 at 60 fps, and feeds the
left of the tracker's two side-by-side sensor images to the mouth model, as the Babble App does.

- The tracker is picked automatically when no Babble tracker is plugged in. It also appears on the USB Camera page
  of the Babble tab, where rotation and flips can be set if the picture is the wrong way up.
- Versions follow the [finaea fork](https://github.com/finaea/Babble-Frame-Runtime) scheme: `V1.2 Release 101` is
  the first fork build on upstream `V1.2 Release`. The updater in the Babble tab follows this fork's releases.
- The fork keeps itself current: the `Sync upstream` workflow checks for new upstream releases every six hours,
  merges each one into `main` and publishes a fork build of it, so updates arrive in the Babble tab as usual. Only
  a merge conflict needs a person.

Install it with the upstream one-liner pointed at this repository:

```
curl -fsSL https://raw.githubusercontent.com/Vamprigamer005/Babble-Frame-Runtime/main/tools/get.sh | BABBLE_REPO=Vamprigamer005/Babble-Frame-Runtime bash
```
