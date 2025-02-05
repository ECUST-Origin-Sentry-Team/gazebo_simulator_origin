#include <iostream>
#include <gz/plugin/Register.hh>
#include <string>
#include "RefereeUI.hh"
#include <cstring>
#include <QSharedMemory>
using namespace gz;
using namespace gui;

RefereeUI::RefereeUI() : Plugin() {}
RefereeUI::~RefereeUI()
{
    if (shared_memory.isAttached())
    {
        shared_memory.detach();
    }
}
void RefereeUI::LoadConfig(const tinyxml2::XMLElement *_pluginElem)
{
    if (!_pluginElem)
        return;

    /*读取消息*/
    auto labelElem = _pluginElem->FirstChildElement("label");
    auto msgElem = _pluginElem->FirstChildElement("msg");
    auto maxValue = _pluginElem->FirstChildElement("maxValue");
    auto defaultValue = _pluginElem->FirstChildElement("default");
    this->label = labelElem->GetText();

    /*消息更新*/
    if (labelElem && labelElem->GetText())
    { // 设置标签名
        if (maxValue && maxValue->GetText())
        { // 设置最大值
            this->maxValue = std::stod(maxValue->GetText());
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  " + std::string(maxValue->GetText())).c_str();
        }
        else
            this->labelScreen = (std::string(labelElem->GetText()) + "  /  100").c_str();
        emit LabelChanged();
        emit MaxValueChanged();
    }
    if (defaultValue && defaultValue->GetText())
    { // 设置默认值
        this->defaultValue = std::stod(defaultValue->GetText());
    }
    else
        this->defaultValue = this->maxValue;
    emit defaultValueChanged();

    // 创建共享内存
    if (shared_memory.isAttached())
    {
        shared_memory.detach(); // 如果已经附加，先分离
    }

    if (msgElem && msgElem->GetText())
    { // 设置消息名
        shared_memory.setKey(QString::fromStdString(std::string(msgElem->GetText())));
    }
    else
    {
        shared_memory.setKey(QString::fromStdString(std::string(labelElem->GetText())));
    }
    if (!shared_memory.create(sizeof(int)))
    {
        std::cerr << "Failed to create shared memory: " << shared_memory.errorString().toStdString() << std::endl;
    }

    std::cout << std::string(labelElem->GetText()) << "\tMax Value:\t" << this->maxValue << "\tDefault Value:\t" << this->defaultValue << std::endl;
    // 设置默认值
    shared_memory.lock();
    memcpy(shared_memory.data(), &(this->defaultValue), sizeof(int));
    shared_memory.unlock();
}

void RefereeUI::OnSlider(int _value)
{
    if (shared_memory.isAttached())
    {
        shared_memory.lock();
        memcpy(shared_memory.data(), &_value, sizeof(int));
        shared_memory.unlock();
    }
    else
    {
    }
}

// Register plugin
IGNITION_ADD_PLUGIN(RefereeUI, gz::gui::Plugin);