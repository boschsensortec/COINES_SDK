# SensorAPI Examples - Git Submodules

This directory contains the Git submodules for SensorAPI examples sourced from the official [Bosch Sensortec GitHub repository](https://github.com/boschsensortec).

## Cloning the Repository with Submodules
To clone this repository along with all its submodules, use the following command:

```sh
git clone --recursive https://github.com/boschsensortec/COINES_SDK.git
```

## Initializing and Updating Submodules
If you have already cloned the repository without submodules, you can initialize and update them using:

```sh
git submodule update --init --recursive
```

## Keeping Submodules Updated
To ensure submodules remain up to date when pulling the latest changes, run:

```sh
git pull --recurse-submodules
```

Then, update the submodules using:

```sh
git submodule update --recursive
```

