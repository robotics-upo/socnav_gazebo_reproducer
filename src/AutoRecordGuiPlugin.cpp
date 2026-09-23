#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class AutoRecordGuiPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: AutoRecordGuiPlugin() : GUIPlugin()
    {
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();

      // Subscribe to the topic sent by the WorldPlugin
      this->sub = this->node->Subscribe("~/record_video", &AutoRecordGuiPlugin::OnRecordCmd, this);
    }

    public: virtual ~AutoRecordGuiPlugin() {}

    private: void OnRecordCmd(ConstGzStringPtr &_msg)
    {
      rendering::UserCameraPtr camera = gui::get_active_camera();
      if (!camera) return;

      std::string cmd = _msg->data();

      if (cmd == "stop")
      {
        camera->StopVideo();
        gzmsg << "[AutoRecordGuiPlugin] Video recording STOPPED." << std::endl;
      }
      else
      {
        // El contenido del mensaje contiene la ruta completa del video
        std::string video_filepath = cmd;

        bool success = camera->StartVideo("mp4", video_filepath);
        if (success) {
          gzmsg << "[AutoRecordGuiPlugin] Video recording STARTED -> " << video_filepath << std::endl;
        } else {
          gzerr << "[AutoRecordGuiPlugin] Error occurred while initiating video recording at: " << video_filepath << std::endl;
        }
      }
    }

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
  };

  GZ_REGISTER_GUI_PLUGIN(AutoRecordGuiPlugin)
}
#include "AutoRecordGuiPlugin.moc"