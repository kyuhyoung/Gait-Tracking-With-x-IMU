# -*- coding: utf-8 -*-
# ------------------------------------------------------------------------------
# FEDERAL UNIVERSITY OF UBERLANDIA
# Faculty of Electrical Engineering
# Biomedical Engineering Lab
# ------------------------------------------------------------------------------
# Author: Italo Gustavo Sampaio Fernandes
# Contact: italogsfernandes@gmail.com
# Git: www.github.com/italogfernandes
# ------------------------------------------------------------------------------
# Description:
# ------------------------------------------------------------------------------
import sys
# ------------------------------------------------------------------------------
# PyQt5
from PyQt5.QtWidgets import *
from views import base_qt5 as base
# PyQt4
# ------------------------------------------------------------------------------

# ------------------------------------------------------------------------------


class ExampleApp(QMainWindow, base.Ui_MainWindow):
    def __init__(self, parent=None):
        super(ExampleApp, self).__init__(parent)
        self.setupUi(self)
        self.setup_signals_connections()
        
    def setup_signals_connections(self):
        pass
    
    def closeEvent(self, q_close_event):
        super(self.__class__, self).closeEvent(q_close_event) 


def main():
    app = QApplication(sys.argv)
    form = ExampleApp()
    form.show()
    app.exec_()

if __name__ == "__main__":
    main()
