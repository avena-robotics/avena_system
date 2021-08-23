#include "avena_view/start_system.h"

StartSystem::StartSystem(Ui::AvenaViewWidget *ui_ptr, rclcpp::Node::SharedPtr node_shared_ptr, NodesMap& nodes_map_ref)
    : _ui_ptr(ui_ptr), _node_shared_ptr(node_shared_ptr), _nodes_map_ref(nodes_map_ref)
{
    _launch_file_manager = std::make_shared<LaunchFileManager>();
    addLaunches();

    _detectron_runner = std::make_shared<DetectronRunner>(_ui_ptr);
    connectSlotsToSignals();
}
StartSystem::~StartSystem()
{
    std::cout << __func__ << std::endl;
    std::cout << "Killing opened launch files" << std::endl;
    for (const auto & launch_file : _launch_file_manager->launchFiles())
    {
        std::cout << launch_file.first << std::endl;
        launch_file.second->terminate();
    }

    _detectron_runner->stopDetectron();
}

void StartSystem::connectSlotsToSignals()
{
    connect(_ui_ptr->startDatastoreButton, SIGNAL(clicked(bool)), this, SLOT(startDataStore()));
    connect(_ui_ptr->startDetectButton, SIGNAL(clicked(bool)), this, SLOT(startDetect()));
    connect(_ui_ptr->startRGBDSyncButton, SIGNAL(clicked(bool)), this, SLOT(startRGBDSync()));
    connect(_ui_ptr->startEstimeteShapeButton, SIGNAL(clicked(bool)), this, SLOT(startEstimateShape()));
    connect(_ui_ptr->startGenerateOctomapButton, SIGNAL(clicked(bool)), this, SLOT(startGenerateOctomap()));
    connect(_ui_ptr->startComposeItemsButton, SIGNAL(clicked(bool)), this, SLOT(startComposeItems()));
    connect(_ui_ptr->startParametersServerButton, SIGNAL(clicked(bool)), this, SLOT(startParametersServer()));
    connect(_ui_ptr->startSecurityRGBButton, SIGNAL(clicked(bool)), this, SLOT(startSecurityRGB()));
    connect(_ui_ptr->startTrackingRGBButton, SIGNAL(clicked(bool)), this, SLOT(startTrackingRGB()));

    connect(_ui_ptr->startCameraButton, SIGNAL(clicked(bool)), this, SLOT(startCamera()));
    connect(_ui_ptr->startCanDriversButton, SIGNAL(clicked()), this, SLOT(startCAN()));
    connect(_ui_ptr->startArmControllerButton, SIGNAL(clicked(bool)), this, SLOT(startArmController()));
    connect(_ui_ptr->startBulletButton, SIGNAL(clicked()), this, SLOT(startBullet()));
    connect(_ui_ptr->startGeneratePathButton, SIGNAL(clicked(bool)), this, SLOT(startGeneratePath()));
    connect(_ui_ptr->startGenerateTrajectoryButton, SIGNAL(clicked(bool)), this, SLOT(startGenerateTrajectory()));
}

void StartSystem::addLaunches()
{
    const std::vector<std::string> launch_files_names = {
        "data_store",
        "simple_controller",
        "rgbd_sync",
        "compose_items",
        "estimate_shape",
        "octomap_generator",
        "security_rgb",
        "parameters_server",
        "generate_path",
        "generate_trajectory"
    };

    std::for_each(
        launch_files_names.begin(),
        launch_files_names.end(),
        [this](const std::string& launch_file_name)
        {
            this->_launch_file_manager->addLaunch(launch_file_name);
        }
    );
}

bool StartSystem::startNode(const QString name,const QStringList args)
{
    auto node_it = _nodes_map_ref.find(name.toStdString());
    if(node_it != _nodes_map_ref.end())
    {
        writeToConsole("Node " + name.toStdString() + " seems to be started already", _ui_ptr->startSystemLogConsole);
        return true;
    }

    auto it = _launch_file_manager->launchFiles().find(name.toStdString());
    if(it == _launch_file_manager->launchFiles().end())
    {
        writeToConsole("Launch file not found", _ui_ptr->startSystemLogConsole);
        return false;
    }
    it->second->setArguments(args);
    auto post_start_check = [this, name](){
        auto it = this->_nodes_map_ref.find(name.toStdString());
        if(it == this->_nodes_map_ref.end())
        {
            writeToConsole("Node " + name.toStdString() + " doesn't have a heartbeat", _ui_ptr->startSystemLogConsole);
            return false;
        }
        writeToConsole("Node " + name.toStdString() + " have a heartbeat", _ui_ptr->startSystemLogConsole);
        return true;
    };
    writeToConsole("Waiting 3 seconds for " + name.toStdString() + " to initialize", _ui_ptr->startSystemLogConsole);
    return it->second->run(post_start_check, 3000);
}

void StartSystem::startCamera()
{
    //TODO: ask Rafal which launch file should I run 
    writeToConsole("Starting cameras isn't done yet.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startCAN()
{
    writeToConsole("Starting CAN isn't done yet.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startArmController()
{
    writeToConsole("Starting simple_controller node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("simple_controller", {"launch", "custom_controllers", "simple_controller.launch.py"});
    if(starting_result){
        // _ui_ptr->startArmControllerButton->setEnabled(false);
        writeToConsole("Successfully started simple_controller node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running simple_controller node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startBullet()
{
    writeToConsole("Starting bullet_server node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("bullet_server", {"launch", "bullet_server", "bullet_server.launch.py"});
    if(starting_result){
        // _ui_ptr->startBulletButton->setEnabled(false);
        writeToConsole("Successfully started bullet_server node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running data_store node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startGeneratePath()
{
    writeToConsole("Starting generate_path node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("generate_path", {"launch", "generate_path", "generate_path.launch.py"});
    if(starting_result){
        // _ui_ptr->startDatastoreButton->setEnabled(false);
        writeToConsole("Successfully started generate_path node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running generate_path node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startGenerateTrajectory()
{
    writeToConsole("Starting generate_trajectory node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("generate_trajectory", {"launch", "generate_trajectory", "generate_trajectory.launch.py"});
    if(starting_result){
        // _ui_ptr->startDatastoreButton->setEnabled(false);
        writeToConsole("Successfully started generate_trajectory node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running generate_trajectory node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startVisionSystem()
{
    writeToConsole("Start System button is not implemented yet", _ui_ptr->startSystemLogConsole);
    // _ui_ptr->startSystemButton->setEnabled(false);
}

void StartSystem::startParametersServer()
{
    writeToConsole("Starting parameters_server node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("parameters_server", {"launch", "parameters_server", "parameters_server.launch.py"});
    if(starting_result){
        // _ui_ptr->startDatastoreButton->setEnabled(false);
        writeToConsole("Successfully started parameters_server node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running parameters_server node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startDataStore()
{
    writeToConsole("Starting data store node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("data_store", {"launch", "data_store", "data_store.launch.py"});
    if(starting_result){
        // _ui_ptr->startDatastoreButton->setEnabled(false);
        writeToConsole("Successfully started data_store node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running data_store node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}
void StartSystem::startDetect()
{
    _detectron_runner->startDetectron();
}

void StartSystem::startSecurityRGB()
{
    writeToConsole("Starting security_rgb node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("security_rgb", {"launch", "security_rgb", "security_rgb.launch.py"});
    if(starting_result){
        // _ui_ptr->startSecurityRGBButton->setEnabled(false);
        writeToConsole("Successfully started security_rgb node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running security_rgb node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startTrackingRGB()
{
    writeToConsole("Tracking RGB is not implemented yet", _ui_ptr->startSystemLogConsole);
}

void StartSystem::startRGBDSync()
{
    writeToConsole("Starting rgdb_sync node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("rgbd_sync", {"launch", "rgbd_sync", "rgbd_sync.launch.py"});
    if(starting_result){
        // _ui_ptr->startRGBDSyncButton->setEnabled(false);
        writeToConsole("Successfully started rgdb_sync node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running rgdb_sync node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}
void StartSystem::startComposeItems()
{
    writeToConsole("Starting compose_items node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("compose_items", {"launch", "compose_items", "compose_items_server.launch.py"});
    if(starting_result){
        // _ui_ptr->startComposeItemsButton->setEnabled(false);
        writeToConsole("Successfully started compose_items node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running compose_items node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}
void StartSystem::startEstimateShape()
{
    writeToConsole("Starting estimate_shape node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("estimate_shape", {"launch", "estimate_shape", "estimate_shape.launch.py"});
    if(starting_result){
        // _ui_ptr->startEstimeteShapeButton->setEnabled(false);
        writeToConsole("Successfully started estimate_shape node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running estimate_shape node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}
void StartSystem::startGenerateOctomap()
{
    writeToConsole("Starting octomap_generator node", _ui_ptr->startSystemLogConsole);
    bool starting_result = startNode("octomap_generator", {"launch", "octomap_generator", "octomap_generator.launch.py"});
    if(starting_result){
        // _ui_ptr->startGenerateOctomapButton->setEnabled(false);
        writeToConsole("Successfully started octomap_generator node", _ui_ptr->startSystemLogConsole);
    }
    else
        writeToConsole("Error while running octomap_generator node. Check terminal for more complex informations.", _ui_ptr->startSystemLogConsole);
}