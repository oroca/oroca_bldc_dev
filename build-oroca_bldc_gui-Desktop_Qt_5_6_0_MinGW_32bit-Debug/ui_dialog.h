/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created by: Qt User Interface Compiler version 5.6.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDial>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFormLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QTabWidget *tabWidget;
    QWidget *tab;
    QDial *dial;
    QLabel *label_8;
    QPushButton *Btn_Run;
    QPushButton *Btn_Run_2;
    QWidget *tab_2;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QFormLayout *formLayout;
    QComboBox *portBox;
    QLabel *label_2;
    QComboBox *baudRateBox;
    QLabel *label_3;
    QComboBox *dataBitsBox;
    QLabel *label_4;
    QComboBox *parityBox;
    QLabel *label_5;
    QComboBox *stopBitsBox;
    QLabel *label_7;
    QSpinBox *timeoutBox;
    QLabel *label_6;
    QComboBox *queryModeBox;
    QLabel *label;
    QPushButton *openCloseButton;
    QTextEdit *textEdit;
    QLabel *label_13;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QStringLiteral("Dialog"));
        Dialog->resize(1182, 692);
        tabWidget = new QTabWidget(Dialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 10, 891, 651));
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        dial = new QDial(tab);
        dial->setObjectName(QStringLiteral("dial"));
        dial->setGeometry(QRect(30, 20, 191, 171));
        label_8 = new QLabel(tab);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(150, 190, 56, 12));
        Btn_Run = new QPushButton(tab);
        Btn_Run->setObjectName(QStringLiteral("Btn_Run"));
        Btn_Run->setGeometry(QRect(20, 230, 91, 23));
        Btn_Run_2 = new QPushButton(tab);
        Btn_Run_2->setObjectName(QStringLiteral("Btn_Run_2"));
        Btn_Run_2->setGeometry(QRect(124, 230, 91, 23));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        tabWidget->addTab(tab_2, QString());
        widget = new QWidget(Dialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(910, 30, 258, 651));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QStringLiteral("formLayout"));
        portBox = new QComboBox(widget);
        portBox->setObjectName(QStringLiteral("portBox"));

        formLayout->setWidget(0, QFormLayout::FieldRole, portBox);

        label_2 = new QLabel(widget);
        label_2->setObjectName(QStringLiteral("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        baudRateBox = new QComboBox(widget);
        baudRateBox->setObjectName(QStringLiteral("baudRateBox"));

        formLayout->setWidget(1, QFormLayout::FieldRole, baudRateBox);

        label_3 = new QLabel(widget);
        label_3->setObjectName(QStringLiteral("label_3"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_3);

        dataBitsBox = new QComboBox(widget);
        dataBitsBox->setObjectName(QStringLiteral("dataBitsBox"));

        formLayout->setWidget(2, QFormLayout::FieldRole, dataBitsBox);

        label_4 = new QLabel(widget);
        label_4->setObjectName(QStringLiteral("label_4"));

        formLayout->setWidget(3, QFormLayout::LabelRole, label_4);

        parityBox = new QComboBox(widget);
        parityBox->setObjectName(QStringLiteral("parityBox"));

        formLayout->setWidget(3, QFormLayout::FieldRole, parityBox);

        label_5 = new QLabel(widget);
        label_5->setObjectName(QStringLiteral("label_5"));

        formLayout->setWidget(4, QFormLayout::LabelRole, label_5);

        stopBitsBox = new QComboBox(widget);
        stopBitsBox->setObjectName(QStringLiteral("stopBitsBox"));

        formLayout->setWidget(4, QFormLayout::FieldRole, stopBitsBox);

        label_7 = new QLabel(widget);
        label_7->setObjectName(QStringLiteral("label_7"));

        formLayout->setWidget(5, QFormLayout::LabelRole, label_7);

        timeoutBox = new QSpinBox(widget);
        timeoutBox->setObjectName(QStringLiteral("timeoutBox"));
        timeoutBox->setMinimum(-1);
        timeoutBox->setMaximum(10000);
        timeoutBox->setSingleStep(10);
        timeoutBox->setValue(10);

        formLayout->setWidget(5, QFormLayout::FieldRole, timeoutBox);

        label_6 = new QLabel(widget);
        label_6->setObjectName(QStringLiteral("label_6"));

        formLayout->setWidget(6, QFormLayout::LabelRole, label_6);

        queryModeBox = new QComboBox(widget);
        queryModeBox->setObjectName(QStringLiteral("queryModeBox"));

        formLayout->setWidget(6, QFormLayout::FieldRole, queryModeBox);

        label = new QLabel(widget);
        label->setObjectName(QStringLiteral("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);


        verticalLayout->addLayout(formLayout);

        openCloseButton = new QPushButton(widget);
        openCloseButton->setObjectName(QStringLiteral("openCloseButton"));

        verticalLayout->addWidget(openCloseButton);

        textEdit = new QTextEdit(widget);
        textEdit->setObjectName(QStringLiteral("textEdit"));

        verticalLayout->addWidget(textEdit);

        label_13 = new QLabel(widget);
        label_13->setObjectName(QStringLiteral("label_13"));
        label_13->setAlignment(Qt::AlignRight|Qt::AlignTrailing|Qt::AlignVCenter);

        verticalLayout->addWidget(label_13);


        retranslateUi(Dialog);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "Dialog", 0));
        label_8->setText(QApplication::translate("Dialog", "TextLabel", 0));
        Btn_Run->setText(QApplication::translate("Dialog", "Run", 0));
        Btn_Run_2->setText(QApplication::translate("Dialog", "Stop", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("Dialog", "Tab 1", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("Dialog", "Tab 2", 0));
        label_2->setText(QApplication::translate("Dialog", "BaudRate:", 0));
        label_3->setText(QApplication::translate("Dialog", "DataBits:", 0));
        label_4->setText(QApplication::translate("Dialog", "Parity:", 0));
        label_5->setText(QApplication::translate("Dialog", "StopBits:", 0));
        label_7->setText(QApplication::translate("Dialog", "Timeout:", 0));
        timeoutBox->setSuffix(QApplication::translate("Dialog", " ms", 0));
        label_6->setText(QApplication::translate("Dialog", "QueryMode:", 0));
        label->setText(QApplication::translate("Dialog", "Port:", 0));
        openCloseButton->setText(QApplication::translate("Dialog", "Open/Close", 0));
        label_13->setText(QApplication::translate("Dialog", "localTime", 0));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
