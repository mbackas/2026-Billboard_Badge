"""App Switcher"""

import asyncio as aio  # type: ignore
import os
import sys
import btree

from apps.base_app import BaseApp
from lib.button import MenuMinus, MenuPlus, UserA, UserB, PRESSED_READ
from lib.led_bar import leds

# MENU_TIMEOUT_MILLISECONDS = 1000


APPMOD_DENYLIST = {
    "__init__",
    "base_app",
    "template",
}


class AppSwitcher:
    def __init__(self, initial_idx: int = 0) -> None:
        self.apps = []
        self.active_idx = initial_idx
        self.selecting_mode = True
        self.selection_idx = initial_idx

        # Discover and load apps
        app_mods = []
        for filename, _, _, _ in os.ilistdir("/apps"):  # pyright: ignore[reportAssignmentType]
            if not filename.endswith(".py") or filename.startswith("_"):
                continue
            app_modname = filename.split(".")[0]
            app_mods.append(app_modname)
        if app_mods:
            print(f"Found app mods: {app_mods}")
            with open("/data/user_apps.py", "w") as out:
                out.write(f"from apps import {', '.join(app_mods)}\n")

        import data.user_apps as apps  # pyright: ignore[reportMissingImports]
        apps_list = [app for app in dir(apps) if "__" not in app]
        print(f"App module list: {apps_list}")
        for appmod_name in apps_list:
            if appmod_name.startswith("_") or appmod_name in APPMOD_DENYLIST:
                continue
            appmod = getattr(apps, appmod_name)
            try:
                for modattr in appmod.__dict__.values():
                    # Subclasses of an app class count too. _abstract skips the
                    # shared base so only the concrete apps are constructed.
                    if not isinstance(modattr, type) or not issubclass(modattr, BaseApp):
                        continue
                    if modattr is BaseApp or modattr.__dict__.get("_abstract"):
                        continue
                    app = modattr()
                    self.apps.append(app)
                    print(f"Added app: {app.name}")
            except Exception as exc:
                print(f"Failed to load app module: {appmod}")
                sys.print_exception(exc)

        try:
            apps_file = open("/data/app_list", "r+b")
        except OSError:
            apps_file = open("/data/app_list", "w+b")
        # Now open a database itself
        apps_db = btree.open(apps_file)
        for idx, app in enumerate(self.apps):
            apps_db[bytes([idx])] = bytes(app.name, "UTF-8")
        apps_db.flush()
        apps_db.close()
        apps_file.close()

        # An app with autostart = True opens at power-up instead of the menu
        # (Menu +/- still leads to the others).
        for idx, app in enumerate(self.apps):
            if getattr(app, "autostart", False):
                print(f"Starting {app.name} (autostart)")
                self.active_idx = self.selection_idx = idx
                app.launched = True
                self.selecting_mode = False
                break


    async def arun(self):
        while True:
            if self.selecting_mode:
               
                ## Display current app from all installed apps
                leds.menu_display(self.selection_idx, len(self.apps))

                if MenuPlus.pressed():
                    self.selection_idx = (self.selection_idx + 1) % len(self.apps)
                    print(
                        f"Selected app {self.selection_idx}: {self.apps[self.selection_idx].name}"
                    )
                if MenuMinus.pressed():
                    self.selection_idx = (self.selection_idx - 1) % len(self.apps)
                    print(
                        f"Selected app {self.selection_idx}: {self.apps[self.selection_idx].name}"
                    )
                if UserA.pressed() or UserB.pressed():
                    print(f"Switching to {self.apps[self.selection_idx].name}")
                    self.active_idx = self.selection_idx
                    self.apps[self.active_idx].launched = True
                    leds.clear_sequence()
                    leds.fill((0, 0, 0))
                    leds.write()
                    self.selecting_mode = False
                await aio.sleep_ms(10)
            else:
                if MenuMinus.pressed() or MenuPlus.pressed():
                    self.selection_idx = self.active_idx
                    self.selecting_mode = True
                    ## Reset pressed state so that button doesn't advance menu, just enters menu state
                    MenuMinus._pressed_state = PRESSED_READ
                    MenuPlus._pressed_state = PRESSED_READ
                    print("Activated app switching")
                await self.apps[self.active_idx].arun()
                await aio.sleep_ms(0)
                self.apps[self.active_idx].launched = False


switcher = AppSwitcher()
