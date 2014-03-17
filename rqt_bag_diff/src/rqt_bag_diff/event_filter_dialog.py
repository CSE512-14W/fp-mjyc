# import python_qt_binding.QtCore as QtCore
import python_qt_binding.QtGui as QtGui


class EventFilterDialog(QtGui.QDialog):
    def __init__(self, parent=None):
        super(EventFilterDialog, self).__init__(parent)

        nameLabel = QtGui.QLabel("Event Filter Name:")
        self.nameLineEdit = QtGui.QLineEdit()

        topicLabel = QtGui.QLabel("Topic:")
        self.topicComboBox = QtGui.QComboBox()
        # self.topicComboBox.addItems(self._timeline._get_topics())
        self.topicComboBox.addItems(self.parent()._timeline_1._get_topics())

        formulaLabel = QtGui.QLabel("Formula:")
        self.formulaLineEdit = QtGui.QLineEdit()
        self.formulaLineEdit.setText('lambda msg,t,prev_val: ')

        windowLabel = QtGui.QLabel("Event Window Length (in sec):")
        self.windowLineEdit = QtGui.QLineEdit()
        # TODO: make sure to not allow 0
        self.windowLineEdit.setValidator(QtGui.QDoubleValidator(0, 999, 2, self.windowLineEdit))


        self.groupBox = QtGui.QGroupBox("Advanced Event Filter")
        self.groupBox.setCheckable(True)
        self.groupBox.setChecked(False)

        updateLabel = QtGui.QLabel("Previous Value Update Formula:")
        self.updateLineEdit = QtGui.QLineEdit()
        self.updateLineEdit.setText('lambda msg,t,prev_val: ')

        initLabel = QtGui.QLabel("Initial Value:")
        self.initLineEdit = QtGui.QLineEdit()
        # TODO: make sure to not allow 0
        self.initLineEdit.setValidator(QtGui.QDoubleValidator(0, 999, 2, self.initLineEdit))

        self.stateCheckbox = QtGui.QCheckBox("Use State Filter")
        self.stateCheckbox.setChecked(False)
        self.stateCheckbox.toggled.connect(self._toggleStateMode)

        vbox = QtGui.QVBoxLayout()
        vbox.addWidget(updateLabel)
        vbox.addWidget(self.updateLineEdit)
        vbox.addWidget(initLabel)
        vbox.addWidget(self.initLineEdit)
        vbox.addWidget(self.stateCheckbox)
        self.groupBox.setLayout(vbox)


        buttonBox = QtGui.QDialogButtonBox(QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel)
        buttonBox.accepted.connect(self._accept)
        buttonBox.rejected.connect(self._reject)


        mainLayout = QtGui.QVBoxLayout()
        mainLayout.addWidget(nameLabel)
        mainLayout.addWidget(self.nameLineEdit)
        mainLayout.addWidget(topicLabel)
        mainLayout.addWidget(self.topicComboBox)
        mainLayout.addWidget(formulaLabel)
        mainLayout.addWidget(self.formulaLineEdit)
        mainLayout.addWidget(windowLabel)
        mainLayout.addWidget(self.windowLineEdit)
        mainLayout.addWidget(self.groupBox)
        mainLayout.addWidget(buttonBox)

        self.setLayout(mainLayout)
        self.setWindowTitle("Add Event Filter")

    def _toggleStateMode(self):
        if self.stateCheckbox.isChecked():
            self.windowLineEdit.setEnabled(False)
        else:
            self.windowLineEdit.setEnabled(True)

    def _accept(self):
        name = self.nameLineEdit.text()
        topic = str(self.topicComboBox.currentText())
        # TODO: syntax check & error report here
        formula = eval(self.formulaLineEdit.text())
        window_len = float(self.windowLineEdit.text())
        update_formula = None
        init_val = None
        if self.groupBox.isChecked():
            update_formula = eval(self.updateLineEdit.text())
            init_val = eval(self.initLineEdit.text())

        is_state = self.stateCheckbox.isChecked() # False

        # self._timeline.create_new_event_filter(name,topic,formula,window_len)
        self.parent()._timeline_1.create_new_event_filter(name,topic,formula,window_len, update_formula, init_val, is_state)
        self.parent()._timeline_2.create_new_event_filter(name,topic,formula,window_len, update_formula, init_val, is_state)

        self.parent().event_selector_1.addItem(name)
        self.parent().event_selector_2.addItem(name)

        # eventnames = self._timeline_1.event_filters.keys()
        # self.event_selector_1.setEnabled(True)
        # self.event_selector_1.addItems(eventnames)
        # from PyQt4.QtCore import pyqtRemoveInputHook
        # from ipdb import set_trace
        # pyqtRemoveInputHook()
        # set_trace()
        self.close()

    def _reject(self):
        self.close()


if __name__ == '__main__':

    import sys

    app = QtGui.QApplication(sys.argv)
    dialog = EventFilterDialog()
    sys.exit(dialog.exec_())
