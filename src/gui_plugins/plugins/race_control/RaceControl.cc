#include <iostream>
#include <gz/plugin/Register.hh>
#include <string>
#include "RaceControl.hh"
#include <cstring>
#include <QSharedMemory>

using namespace gz;
using namespace gui;

RaceControl::RaceControl() : Plugin(), timeMemory(), bulletMemory() {}

RaceControl::~RaceControl()
{
    if (timeMemory.isAttached())
        timeMemory.detach();
    if (bulletMemory.isAttached())
        bulletMemory.detach();
}

void RaceControl::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
    if (!_pluginElem)
        return;

    auto labelElem = _pluginElem->FirstChildElement("label");
    auto msgElem = _pluginElem->FirstChildElement("msg");
    auto maxValue = _pluginElem->FirstChildElement("maxValue");
    auto defaultValue = _pluginElem->FirstChildElement("default");

    this->label = labelElem->GetText();

    if (labelElem && labelElem->GetText())
    {
        if (maxValue && maxValue->GetText())
        {
            this->maxValue = std::stod(maxValue->GetText());
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  " + std::string(maxValue->GetText())).c_str();
        }
        else
        {
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  100").c_str();
        }
        emit LabelChanged();
        emit MaxValueChanged();
    }

    if (defaultValue && defaultValue->GetText())
        this->defaultValue = std::stod(defaultValue->GetText());
    else
        this->defaultValue = this->maxValue;

    emit defaultValueChanged();
    emit TimeChanged();
    emit BulletAbleSendChanged();

    // Setup time memory
    if (timeMemory.isAttached())
        timeMemory.detach();

    timeKey = msgElem && msgElem->GetText() ? msgElem->GetText() : labelElem->GetText();
    timeMemory.setKey(QString::fromStdString(timeKey));

    if (!timeMemory.create(sizeof(int)))
    {
        std::cerr << "Failed to create time shared memory: " << timeMemory.errorString().toStdString() << std::endl;
    }
    else
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &(this->defaultValue), sizeof(int));
        timeMemory.unlock();
    }

    std::cout << std::string(labelElem->GetText()) << "\tMax Value:\t" << this->maxValue << "\tDefault Value:\t" << this->defaultValue << std::endl;
}

void RaceControl::OnSlider(int _value)
{
    int minutes = _value / 60;
    int seconds = _value % 60;
    this->bulletablesend = ((6 - minutes) > 0 ? (6 - minutes) : 0) * 100 - this->bulletsended;

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%02d:%02d", minutes, seconds);
    this->simTimeStr = buffer;

    if (timeMemory.isAttached())
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &_value, sizeof(int));
        timeMemory.unlock();
    }

    emit TimeChanged();
    emit BulletAbleSendChanged();
}

void RaceControl::resetSlider()
{

    this->simTimeStr = {"07:00"};
    this->bulletablesend = 0;
    this->bulletsended = 0;

    if (!timeMemory.isAttached())
    {
        timeMemory.setKey(QString::fromStdString(this->timeKey));
        if (!timeMemory.attach())
        {
            std::cerr << "[RaceControl] Failed to attach to time memory: "
                      << timeMemory.errorString().toStdString() << std::endl;
        }
    }
    if (timeMemory.isAttached())
    {
        timeMemory.lock();
        memcpy(timeMemory.data(), &(this->defaultValue), sizeof(int));
        timeMemory.unlock();
        std::cout << "[RaceControl] Reset shared memory to default value: "
                  << this->defaultValue << std::endl;
    }

    updateBullet(300, 0);

    emit BulletAbleSendChanged();
    emit TimeChanged();
    emit defaultValueChanged();
}

void RaceControl::sendBullet()
{
    updateBullet(this->bulletablesend, 1);
    emit BulletAbleSendChanged();
}

void RaceControl::updateBullet(int bulletosend, int flag)
{
    bulletMemory.setKey("bullet_remaining_num_17mm");
    if (!bulletMemory.isAttached())
    {
        if (!bulletMemory.attach())
        {
            std::cerr << "[RaceControl] Failed to attach to bullet memory: "
                      << bulletMemory.errorString().toStdString() << std::endl;
            return;
        }
    }

    if (!bulletMemory.constData())
    {
        std::cerr << "[RaceControl] Bullet memory constData is null!" << std::endl;
        return;
    }

    bulletMemory.lock();
    int sendingbullet = bulletosend + (flag ? *reinterpret_cast<const int *>(bulletMemory.constData()) : 0);
    memcpy(bulletMemory.data(), &(sendingbullet), sizeof(int));
    bulletMemory.unlock();

    this->bulletsended += this->bulletablesend;
    this->bulletablesend = 0;
}

// Register plugin
IGNITION_ADD_PLUGIN(RaceControl, gz::gui::Plugin);
