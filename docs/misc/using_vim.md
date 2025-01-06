# Using Vim #

One thing you'll find yourself doing a lot is editing text files. It is sometimes really handy to be able to edit these files from within the terminal. Vim is an awesome program that you can use to quickly edit text files without leaving the terminal window. It has a great deal of support and some awesome key shortcuts that make editing easy. It does, however, have a pretty steep learning curve. This is to hopefully get you started and at least familiar with the program. If you want more information, you'll have to go to external resources, such as [this one](https://www.fprintf.net/vimCheatSheet.html).

## Installing Vim ##
Installing Vim is just as easy as any other program on Linux

```bash
sudo apt install vim
```

and that's it!

## Editing Files ##
To edit files, all you have to do is type

```bash
vim <textfile>
```

If "textfile" doesn't exist, then vim will create a new blank file and open it for you, otherwise it will open the existing file.

When you first open vim, it's in "navigation mode." In this mode, you can't actually just type stuff like you would normally expect, but you can use the arrow keys to navigate around the document, you can cut and paste, search and replace, and other useful things. If you don't realize that you're not in insert mode, then trying to type will probably result in really random behavior. To enter "insert" mode, type

```bash
i
```

In insert mode, you can now add text, delete text, and edit as you would normally expect. To leave insert mode, just press the Esc button.

To leave the file, there are two mains ways to exit.

First, to write your changes to file and exit,

```bash
:x
```

Second, to quit without saving changes

```bash
:q!
```

And that's all! Again, this is just barely scratching the surface of what vim can do. I would recommend looking through some of the keyboard shortcuts, and watching or reading some tutorials online. Knowing these shortcuts can really speed up your work, much faster than using a standard graphics-based editor like gedit or notepad++.
