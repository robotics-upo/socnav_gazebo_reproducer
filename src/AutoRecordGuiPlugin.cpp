#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <QCoreApplication>
#include <QMetaObject>
#include <QTimer>

namespace gazebo
{
  namespace
  {
    constexpr unsigned int kVideoWidth = 1560;
    constexpr unsigned int kVideoHeight = 886;
  }

  class AutoRecordGuiPlugin : public GUIPlugin
  {
    Q_OBJECT

    public: AutoRecordGuiPlugin() : GUIPlugin()
    {
      gzmsg << "[AutoRecordGuiPlugin] Loading GUI recording plugin." << std::endl;
      this->node = transport::NodePtr(new transport::Node());
      this->node->Init();
      this->status_pub = this->node->Advertise<msgs::GzString>("~/record_video_status");

      // Subscribe to the topic sent by the WorldPlugin
      this->sub = this->node->Subscribe("~/record_video", &AutoRecordGuiPlugin::OnRecordCmd, this);
      gzmsg << "[AutoRecordGuiPlugin] Subscribed to ~/record_video." << std::endl;
      QTimer::singleShot(100, this, SLOT(HideSceneDecorations()));
    }

    public: virtual ~AutoRecordGuiPlugin() {}

    private: void OnRecordCmd(ConstGzStringPtr &_msg)
    {
      gzmsg << "[AutoRecordGuiPlugin] Command received: " << _msg->data() << std::endl;
      if (_msg->data() == "stop")
      {
        this->stop_pending = true;
      }
      else
      {
        if (this->recording)
        {
          gzmsg << "[AutoRecordGuiPlugin] Start command ignored: recording is already active."
                << std::endl;
          return;
        }
        this->pending_start_path = _msg->data();
        this->start_pending = true;
      }

      // Transport callbacks may run outside the Qt GUI thread.  Queue the
      // camera operation so it runs after the GUI has created its camera.
      QMetaObject::invokeMethod(this, "TryPendingCommand", Qt::QueuedConnection);
    }

    private slots: void HideSceneDecorations()
    {
      rendering::UserCameraPtr camera = gui::get_active_camera();
      if (!camera || !camera->GetScene())
      {
        if (++this->decoration_retry_count < 50)
          QTimer::singleShot(100, this, SLOT(HideSceneDecorations()));
        return;
      }

      rendering::ScenePtr scene = camera->GetScene();
      scene->ShowOrigin(false);

      rendering::LightPtr sun = scene->LightByName("sun");
      if (sun)
        sun->ShowVisual(false);
      else if (++this->decoration_retry_count < 50)
        QTimer::singleShot(100, this, SLOT(HideSceneDecorations()));
    }

    private slots: void TryPendingCommand()
    {
      if (!this->start_pending && !this->stop_pending) return;

      rendering::UserCameraPtr camera = gui::get_active_camera();
      if (!camera)
      {
        ++this->retry_count;
        if (this->retry_count == 1 || this->retry_count % 10 == 0)
          gzmsg << "[AutoRecordGuiPlugin] Waiting for active camera (retry "
                << this->retry_count << ")." << std::endl;
        QTimer::singleShot(100, this, SLOT(TryPendingCommand()));
        return;
      }

      if (this->stop_pending)
      {
        if (this->recording)
        {
          bool success = camera->StopVideo();
          this->recording = false;
          this->stop_pending = false;
          if (success)
          {
            bool saved = camera->SaveVideo(this->pending_start_path);
            if (saved)
            {
              gzmsg << "[AutoRecordGuiPlugin] Video saved to: "
                    << this->pending_start_path << std::endl;
              msgs::GzString status;
              status.set_data("stopped");
              this->status_pub->Publish(status);
              gzmsg << "[AutoRecordGuiPlugin] Finalization acknowledgment sent."
                    << std::endl;
              QTimer::singleShot(1500, this, SLOT(ShutdownClient()));
            }
            else
            {
              gzerr << "[AutoRecordGuiPlugin] SaveVideo failed for: "
                    << this->pending_start_path << std::endl;
            }
          }
          else
            gzerr << "[AutoRecordGuiPlugin] StopVideo failed; MP4 may be incomplete."
                  << std::endl;
          return;
        }

        if (!this->start_pending) this->stop_pending = false;
      }

      if (!this->start_pending) return;

      camera->SetImageSize(kVideoWidth, kVideoHeight);
      gzmsg << "[AutoRecordGuiPlugin] Configured video size: "
            << camera->GetImageWidth() << "x" << camera->GetImageHeight()
            << " (requested " << kVideoWidth << "x" << kVideoHeight << ")."
            << std::endl;

      // StartVideo writes to Gazebo's temporary encoder buffer. SaveVideo()
      // below copies the finalized stream to the requested output path.
      bool success = camera->StartVideo("mp4");
      if (!success)
      {
        ++this->retry_count;
        if (this->retry_count == 1 || this->retry_count % 10 == 0)
          gzerr << "[AutoRecordGuiPlugin] StartVideo failed for: "
                << this->pending_start_path << " (retry " << this->retry_count << ")."
                << std::endl;
        QTimer::singleShot(100, this, SLOT(TryPendingCommand()));
        return;
      }

      this->recording = true;
      this->start_pending = false;
      this->retry_count = 0;
      msgs::GzString status;
      status.set_data("started");
      this->status_pub->Publish(status);
      gzmsg << "[AutoRecordGuiPlugin] Video recording STARTED -> "
        << this->pending_start_path << std::endl;
      gzmsg << "[AutoRecordGuiPlugin] Frames are now being captured from the active camera."
        << std::endl;

      if (this->stop_pending)
        QTimer::singleShot(100, this, SLOT(TryPendingCommand()));
    }

    private slots: void ShutdownClient()
    {
      gzmsg << "[AutoRecordGuiPlugin] Video finalized; shutting down Gazebo client."
            << std::endl;
      gazebo::client::shutdown();
      QCoreApplication::quit();
    }

    private: transport::NodePtr node;
    private: transport::SubscriberPtr sub;
    private: transport::PublisherPtr status_pub;
    private: std::string pending_start_path;
    private: bool start_pending = false;
    private: bool stop_pending = false;
    private: bool recording = false;
    private: unsigned int retry_count = 0;
    private: unsigned int decoration_retry_count = 0;
  };

  GZ_REGISTER_GUI_PLUGIN(AutoRecordGuiPlugin)
}
#include "AutoRecordGuiPlugin.moc"