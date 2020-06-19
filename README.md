# Gateway DFU
----------------------------------------

## Description
I'm sure someone can describe the project better than me so this is a placeholder for that poor soul.

## Setup

0. If you haven't setup a project with Segger Embedded Studio/ NRF Connect before, first install the Getting Started Assistant in nRF Connect. Follow the instructions until you get to the "Initialze west and clone the nRF Connect SDK..." step. Then come back here.

1. Run init.bat from the folder that it is in (the root of this repository). This step well setup ncs and friends correctly for this project as this project uses an apricity modified version of nrf.
2. (Only if you did step 0). Continue with the rest of the Getting Started Assistant steps, starting with the last step in the 2nd section (The nRF connect SDK Zephyr repository contains a list of required Python modules....) Complete the rest of the sections as well.
3. Make sure your zephyr base is pointed to this repositorie's version of zephyr. This is done in section 4 of the assistant if you went through that in the previous step.
4. Import the project. The options will look like the image below. NOTE: Your root directory will VERY LIKELY be different. Just keep that in mind, I'm sure you can figure that out.

![ImportSettings](documentation/good_project_import_settings.png)
