/*
 * Copyright (C) 2017 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */
#ifndef GZ_GUI_RaceControl_HH_
#define GZ_GUI_RaceControl_HH_

#include <string>
#include <gz/gui/qt.h>
#include <gz/gui/Plugin.hh>
#include <QSharedMemory>
namespace ignition
{
  namespace gui
  {
    class RaceControl : public Plugin
    {
      Q_OBJECT
      Q_PROPERTY(QString label READ Label NOTIFY LabelChanged);
      Q_PROPERTY(int maxValue READ MaxValue NOTIFY MaxValueChanged);
      Q_PROPERTY(int defaultValue READ DefaultValue NOTIFY defaultValueChanged);
      Q_PROPERTY(int bulletablesend READ BulletAbleSend NOTIFY BulletAbleSendChanged);
      Q_PROPERTY(QString time READ Time NOTIFY TimeChanged);

    public:
      RaceControl();
      virtual ~RaceControl();
      virtual void LoadConfig(const tinyxml2::XMLElement *_pluginElem);
      Q_INVOKABLE QString Label() const { return QString::fromStdString(labelScreen); }
      Q_INVOKABLE QString Time() const { return QString::fromStdString(simTimeStr); }
      Q_INVOKABLE int MaxValue() const { return maxValue; }
      Q_INVOKABLE int DefaultValue() const { return defaultValue; }

      Q_INVOKABLE int BulletAbleSend() const { return bulletablesend; }
      Q_INVOKABLE void resetSlider();
      Q_INVOKABLE void sendBullet();

    signals:
      void LabelChanged();
      void MaxValueChanged();
      void defaultValueChanged();
      void MsgChanged();
      void TimeChanged();
      void BulletAbleSendChanged();

    protected slots:
      void OnSlider(int _value);

    private:
      std::string label{"Default Name"};
      std::string labelScreen{"Default Name"};
      std::string simTimeStr{"07:00"};
      int maxValue = 100;
      int defaultValue = 100;
      int bulletablesend = 0;
      int bulletsended = 0;
      QSharedMemory timeMemory;
      QSharedMemory bulletMemory;
      void updateBullet(int bulletosend, int flag);
      std::string timeKey;
    };
  }
}

#endif