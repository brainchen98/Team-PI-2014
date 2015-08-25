*NOTE: THIS IS NOW OUTDATED! All the code here was for 2014*

Team-PI
=======

**Team PI Robotics - Brisbane Boys' College**

Robocup Junior Open Soccer

[nonterminating.com](http://nonterminating.com/)

###Team Members
Brian Chen, Robert Kopel, Andrew Su

###Past Results
**2013**
- Robocup Junior Australia 3rd place
- Robocup Junior Queensland 2nd place

Qualified to represent Australia in the Junior competition of Robocup in Brazil 2014.

**2014**
- Robocup Junior Regionals 1st place
- Robocup Junior Queensland 1st place
- Robocup Junior Australia 1st place

~~Qualified to represent Australia in the Junior competition of Robocup in Hefei, China 2015.~~

Represented Australia in Robocup In Hefei, China 2015!

**2015**
- Robocup Junior Internationals 1st place (superteam challenge)
- Robocup Junior Internationals 8th place (individual performance)
- Robocup Junior Queensland 1st place

###Our Goal(s)
- ~~To achieve 1st in Robocup Internationals~~ (we've now kind of achieved that, though only in half the comp.)
- To achieve 1st in Robocup Australia (again)

###Software
Use Sublime Text 3 - it's intuitive and looks good.
Suggested packages
- [Package Control](https://sublime.wbond.net/)! The most important plugin you'll ever need for Sublime.
- Stino (you can install the dev/beta version from a the [new-stino](https://github.com/Robot-Will/Stino/tree/new-stino) branch on Github
- [Sublimelinter](http://www.sublimelinter.com/en/latest/)
- [Sublimelinter-cppcheck](https://github.com/SublimeLinter/SublimeLinter-cppcheck) (works with Arduino if you set the right settings). Make sure you install [Cppcheck](http://cppcheck.sourceforge.net/) beforehand. If you're having trouble getting it even linting c++ sources, then see [this](http://cppstartingkitproject-guide.readthedocs.org/en/latest/C++_Starting_Kit_Plugin--Recommandation--Prerequisite--Install_Cppcheck--Test--Minimal_Set_Up.html?highlight=linter). Once you get the linting on .cpp/.h files working, make sure you change you're Sublimelinter preferences to:

        "syntax_map": {
            "Arduino": "c_cppcheck",
            "C": "c_cppcheck",
            "C++": "c_cppcheck",
            "html (django)": "html",
            "html (rails)": "html",
            "html 5": "html",
            "php": "html",
            "python django": "python"
        }
(or something similar)

- If you want a light colour theme, get the [Visual Studio Theme](https://github.com/mihaifm/Visual-Studio.tmTheme)
*update. Get [ColorSublime](http://colorsublime.com/) from package control to choose your favourite colour theme real time!
I'm currently using Xcode_default
- Regardless of what colour theme you want, you must get the [Spacegray Themes](http://kkga.github.io/spacegray/)
*update. Get [Orchis Light](https://packagecontrol.io/packages/Theme%20-%20Orchis) from package control. It's much better and flatter (and minimalistic!) Much more suited to a light theme.

