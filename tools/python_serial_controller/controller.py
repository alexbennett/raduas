#!/bin/python
import wx

class windowClass(wx.Frame):

    def __init__(self, *args, **kwargs):
        super(windowClass, self).__init__(*args, **kwargs)

        self.basicGUI()

    def basicGUI(self):
        # Top menu bar
        menuBar = wx.MenuBar()

        # File menu
        fileButton = wx.Menu()
        exitBtn = fileButton.Append(wx.ID_EXIT, 'Exit', 'Exit the PID tuner')

        # Append buttons to main menu
        menuBar.Append(fileButton, 'File')

        self.SetMenuBar(menuBar)

        self.Bind(wx.EVT_MENU, self.Quit, exitBtn)

        self.SetTitle('PID Tuner (R.A.D.U.A.S.)')

        self.Show(True)

    def Quit(self, e):
        self.Close()


def main():
    app = wx.App()
    windowClass(None)
    app.MainLoop()

main()
