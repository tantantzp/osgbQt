/********************************************************************************
** Form generated from reading UI file 'osgbqt.ui'
**
** Created by: Qt User Interface Compiler version 5.4.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_OSGBQT_H
#define UI_OSGBQT_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_osgbQtClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *osgbQtClass)
    {
        if (osgbQtClass->objectName().isEmpty())
            osgbQtClass->setObjectName(QStringLiteral("osgbQtClass"));
        osgbQtClass->resize(600, 400);
        menuBar = new QMenuBar(osgbQtClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        osgbQtClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(osgbQtClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        osgbQtClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(osgbQtClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        osgbQtClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(osgbQtClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        osgbQtClass->setStatusBar(statusBar);

        retranslateUi(osgbQtClass);

        QMetaObject::connectSlotsByName(osgbQtClass);
    } // setupUi

    void retranslateUi(QMainWindow *osgbQtClass)
    {
        osgbQtClass->setWindowTitle(QApplication::translate("osgbQtClass", "osgbQt", 0));
    } // retranslateUi

};

namespace Ui {
    class osgbQtClass: public Ui_osgbQtClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_OSGBQT_H
