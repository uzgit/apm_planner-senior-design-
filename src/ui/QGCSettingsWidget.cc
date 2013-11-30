#include <QSettings>

#include "QGCSettingsWidget.h"
#include "MainWindow.h"
#include "ui_QGCSettingsWidget.h"

#include "LinkManager.h"
#include "MAVLinkProtocol.h"
#include "MAVLinkSettingsWidget.h"
#include "GAudioOutput.h"

//, Qt::WindowFlags flags

QGCSettingsWidget::QGCSettingsWidget(QWidget *parent, Qt::WindowFlags flags) :
    QDialog(parent, flags),
    ui(new Ui::QGCSettingsWidget)
{
    m_init = false;
    ui->setupUi(this);

    // Add all protocols
    QList<ProtocolInterface*> protocols = LinkManager::instance()->getProtocols();
    foreach (ProtocolInterface* protocol, protocols) {
        MAVLinkProtocol* mavlink = dynamic_cast<MAVLinkProtocol*>(protocol);
        if (mavlink) {
            MAVLinkSettingsWidget* msettings = new MAVLinkSettingsWidget(mavlink, this);
            ui->tabWidget->addTab(msettings, "MAVLink");
        }
    }

    this->window()->setWindowTitle(tr("QGroundControl Settings"));


}
void QGCSettingsWidget::showEvent(QShowEvent *evt)
{
    if (!m_init)
    {
        m_init = true;
        // Audio preferences
        ui->audioMuteCheckBox->setChecked(GAudioOutput::instance()->isMuted());
        connect(ui->audioMuteCheckBox, SIGNAL(toggled(bool)), GAudioOutput::instance(), SLOT(mute(bool)));
        connect(GAudioOutput::instance(), SIGNAL(mutedChanged(bool)), ui->audioMuteCheckBox, SLOT(setChecked(bool)));

        // Reconnect
        ui->reconnectCheckBox->setChecked(MainWindow::instance()->autoReconnectEnabled());
        connect(ui->reconnectCheckBox, SIGNAL(clicked(bool)), MainWindow::instance(), SLOT(enableAutoReconnect(bool)));

        // Low power mode
        ui->lowPowerCheckBox->setChecked(MainWindow::instance()->lowPowerModeEnabled());
        connect(ui->lowPowerCheckBox, SIGNAL(clicked(bool)), MainWindow::instance(), SLOT(enableLowPowerMode(bool)));

        //Dock widget title bars
        ui->titleBarCheckBox->setChecked(MainWindow::instance()->dockWidgetTitleBarsEnabled());
        connect(ui->titleBarCheckBox,SIGNAL(clicked(bool)),MainWindow::instance(),SLOT(enableDockWidgetTitleBars(bool)));

        ui->logDirEdit->setText(QGC::logDirectory());
        connect(ui->logDirBrowseButton, SIGNAL(clicked()), this, SLOT(browseButtonClicked()));

        // Style
        MainWindow::QGC_MAINWINDOW_STYLE style = (MainWindow::QGC_MAINWINDOW_STYLE)MainWindow::instance()->getStyle();
        switch (style) {
        case MainWindow::QGC_MAINWINDOW_STYLE_NATIVE:
            ui->nativeStyle->setChecked(true);
            break;
        case MainWindow::QGC_MAINWINDOW_STYLE_INDOOR:
            ui->indoorStyle->setChecked(true);
            break;
        case MainWindow::QGC_MAINWINDOW_STYLE_OUTDOOR:
            ui->outdoorStyle->setChecked(true);
            break;
        }
        connect(ui->nativeStyle, SIGNAL(clicked()), MainWindow::instance(), SLOT(loadNativeStyle()));
        connect(ui->indoorStyle, SIGNAL(clicked()), MainWindow::instance(), SLOT(loadIndoorStyle()));
        connect(ui->outdoorStyle, SIGNAL(clicked()), MainWindow::instance(), SLOT(loadOutdoorStyle()));

        // Close / destroy
        //connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(deleteLater()));

        // Set layout options
        ui->generalPaneGridLayout->setAlignment(Qt::AlignTop);
    }
}

QGCSettingsWidget::~QGCSettingsWidget()
{
    delete ui;
}

void QGCSettingsWidget::browseButtonClicked() {
    QFileDialog dlg(this);
    dlg.setFileMode(QFileDialog::Directory);
    dlg.setDirectory(QGC::logDirectory());

    if(dlg.exec()) {
        QDir dir = dlg.directory();
        QString name = dir.absolutePath();
        QGC::setLogDirectory(name);
        ui->logDirEdit->setText(name);
    }
}
