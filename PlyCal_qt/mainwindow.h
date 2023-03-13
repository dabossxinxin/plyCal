#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "tfwindow.h"
#include "pointcloudviewer.h"
#include "imageviewer.h"

#include "json/json.hpp"
#include "calibrator.h"
#include "data_reader.h"

namespace Ui
{
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

public slots:
    void updateWithTransformation(Eigen::Matrix4d tf);
    void tfwindowClose();

protected:
    void closeEvent(QCloseEvent *);
private:
    struct SensorData
    {
        bool img_good;
        bool pc_good;
        uint32_t id;
        uint32_t pid;
        std::shared_ptr<cv::Mat> img;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc;
        std::shared_ptr<cv::Mat> img_marked;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_marked;
        std::shared_ptr<cv::Mat> img_proj;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_proj;

        SensorData(uint32_t index):img_good(false), pc_good(false),
            id(index), img(nullptr), pc(nullptr),
            img_marked(nullptr), pc_marked(nullptr),
            img_proj(nullptr), pc_proj(nullptr){}
        bool good() {
            return (img_good && pc_good);
        }
    };

    Ui::MainWindow *ui;

	QString			data_root_;			// ���ݶ�ȡ·��
    QString			config_path_;		// �����ļ�·��
    nlohmann::json	js_;				// �����ļ�handle

    bool			is_calibrated_;		// �Ƿ��Ѿ��궨
	bool			is_initialized_;	// �Ƿ��Ѿ���ʼ��

    uint32_t		sid_;

    std::shared_ptr<cv::Mat>				img_;			// ͼ������
    pcl::PointCloud<pcl::PointXYZI>::Ptr	pc_;			// ��������
	std::vector<SensorData>					sensor_data_;	// ��Ժ��ͼ���������
    std::unique_ptr<DataReader>				data_reader_;	// ���ݶ�ȡ����ָ��
	std::unique_ptr<lqh::Calibrator>		calibrator_;	// ��α궨����ָ��

    std::unique_ptr<PointcloudViewer>		pc_viewer_;		// �����ӿ�
    std::unique_ptr<ImageViewer>			img_viewer_;	// ͼ���ӿ�
    std::unique_ptr<TFwindow>				tfwindow_;		// ������̬�任����

    void updateLabels();
    bool processData(bool is_check = true);
    void setEnabledAll(bool status);
    void showCalibrateResult();
    void showTFWindow();

private slots:
    void on_actionSet_K_triggered();
    void on_actionSet_D_triggered();
    void on_actionOpen_Dataset_triggered();

    void on_next_pose_clicked();
    void on_quick_next_pose_clicked();
    void on_delete_pose_clicked();
    void on_calibrate_clicked();
	void on_initialize_clicked();

    void on_pick_points_start_clicked();
    void on_pick_points_end_clicked();
    void on_pick_points_quit_clicked();
    void on_actionSave_Result_triggered();

    void processSlider();
    void on_actionSave_Config_triggered();
};

#endif // MAINWINDOW_H
