/*
 * Copyright (C) 2018 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "tasks.h"
#include <stdexcept>
#include <string>

// Déclaration des priorités des taches
#define PRIORITY_TSERVER 30
#define PRIORITY_TOPENCOMROBOT 20
#define PRIORITY_TMOVE 20
#define PRIORITY_TSENDTOMON 22
#define PRIORITY_TRECEIVEFROMMON 25
#define PRIORITY_TSTARTROBOT 20
#define PRIORITY_TCAMERA 21
#define PRIORITY_TBATTERY 18

//Variables globales
int comRobot_FailCounter = 0;
int comRobot_Lost = 0;
/*
 * Some remarks:
 * 1- This program is mostly a template. It shows you how to create tasks, semaphore
 *   message queues, mutex ... and how to use them
 * 
 * 2- semDumber is, as name say, useless. Its goal is only to show you how to use semaphore
 * 
 * 3- Data flow is probably not optimal
 * 
 * 4- Take into account that ComRobot::Write will block your task when serial buffer is full,
 *   time for internal buffer to flush
 * 
 * 5- Same behavior existe for ComMonitor::Write !
 * 
 * 6- When you want to write something in terminal, use cout and terminate with endl and flush
 * 
 * 7- Good luck !
 */

/**
 * @brief Initialisation des structures de l'application (tâches, mutex, 
 * semaphore, etc.)
 */
void Tasks::Init() {
    int status;
    int err;

    /**************************************************************************************/
    /* 	Mutex creation                                                                    */
    /**************************************************************************************/
    if (err = rt_mutex_create(&mutex_monitor, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robot, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_robotStarted, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_move, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_comrobot_failcounter, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_comrobot_lost, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_compteurWD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_modeWD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_mutex_create(&mutex_reloadWD, NULL)) {
        cerr << "Error mutex create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Mutexes created successfully" << endl << flush;

    /**************************************************************************************/
    /* 	Semaphors creation       							  */
    /**************************************************************************************/
    if (err = rt_sem_create(&sem_barrier, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_openComRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_serverOk, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_startRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_stopRobot, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_sem_create(&sem_reloadWD, NULL, 0, S_FIFO)) {
        cerr << "Error semaphore create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Semaphores created successfully" << endl << flush;

    /**************************************************************************************/
    /* Tasks creation                                                                     */
    /**************************************************************************************/
    if (err = rt_task_create(&th_server, "th_server", 0, PRIORITY_TSERVER, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToMon, "th_sendToMon", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_receiveFromMon, "th_receiveFromMon", 0, PRIORITY_TRECEIVEFROMMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_openComRobot, "th_openComRobot", 0, PRIORITY_TOPENCOMROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_startRobot, "th_startRobot", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_move, "th_move", 0, PRIORITY_TMOVE, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_sendToRobot, "th_sendToRobot", 0, PRIORITY_TSENDTOMON, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_battery, "th_battery", 0, PRIORITY_TBATTERY, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_create(&th_reloadWD, "th_reloadWD", 0, PRIORITY_TSTARTROBOT, 0)) {
        cerr << "Error task create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Tasks created successfully" << endl << flush;

    /**************************************************************************************/
    /* Message queues creation                                                            */
    /**************************************************************************************/
    if ((err = rt_queue_create(&q_messageToMon, "q_messageToMon", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if ((err = rt_queue_create(&q_messageToRobot, "q_messageToRobot", sizeof (Message*)*50, Q_UNLIMITED, Q_FIFO)) < 0) {
        cerr << "Error msg queue create: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    cout << "Queues created successfully" << endl << flush;

}

/**
 * @brief Démarrage des tâches
 */
void Tasks::Run() {
    rt_task_set_priority(NULL, T_LOPRIO);
    int err;

    if (err = rt_task_start(&th_server, (void(*)(void*)) & Tasks::ServerTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_sendToMon, (void(*)(void*)) & Tasks::SendToMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_receiveFromMon, (void(*)(void*)) & Tasks::ReceiveFromMonTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_openComRobot, (void(*)(void*)) & Tasks::OpenComRobot, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_startRobot, (void(*)(void*)) & Tasks::StartRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_move, (void(*)(void*)) & Tasks::MoveTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_battery, (void(*)(void*)) & Tasks::BatteryTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    
    if (err = rt_task_start(&th_sendToRobot, (void(*)(void*)) & Tasks::SendToRobotTask, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }
    if (err = rt_task_start(&th_reloadWD, (void(*)(void*)) & Tasks::ReloadWD, this)) {
        cerr << "Error task start: " << strerror(-err) << endl << flush;
        exit(EXIT_FAILURE);
    }

    cout << "Tasks launched" << endl << flush;
}

/**
 * @brief Arrêt des tâches
 */
void Tasks::Stop() {
    monitor.Close();
    robot.Close();
}

/**
 */
void Tasks::Join() {
    cout << "Tasks synchronized" << endl << flush;
    rt_sem_broadcast(&sem_barrier);
    pause();
}

/**
 * @brief Thread handling server communication with the monitor.
 */
void Tasks::ServerTask(void *arg) {
    int status;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are started)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task server starts here                                                        */
    /**************************************************************************************/
    rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
    status = monitor.Open(SERVER_PORT);
    rt_mutex_release(&mutex_monitor);

    cout << "Open server on port " << (SERVER_PORT) << " (" << status << ")" << endl;

    if (status < 0) throw std::runtime_error {
        "Unable to start server on port " + std::to_string(SERVER_PORT)
    };
    monitor.AcceptClient(); // Wait the monitor client
    cout << "Rock'n'Roll baby, client accepted!" << endl << flush;
    rt_sem_broadcast(&sem_serverOk);
}

/**
 * @brief Thread sending data to monitor.
 */
void Tasks::SendToMonTask(void* arg) {
    Message *msg;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);

    /**************************************************************************************/
    /* The task sendToMon starts here                                                     */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);

    while (1) {
        cout << "wait msg to send" << endl << flush;
        msg = ReadInQueue(&q_messageToMon);
        cout << "Send msg to mon: " << msg->ToString() << endl << flush;
        rt_mutex_acquire(&mutex_monitor, TM_INFINITE);
        monitor.Write(msg); // The message is deleted with the Write
        rt_mutex_release(&mutex_monitor);
    }
}

/**
 * @brief Thread receiving data from monitor.
 */
void Tasks::ReceiveFromMonTask(void *arg) {
    Message *msgRcv;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task receiveFromMon starts here                                                */
    /**************************************************************************************/
    rt_sem_p(&sem_serverOk, TM_INFINITE);
    cout << "Received message from monitor activated" << endl << flush;

    while (1) {
        msgRcv = monitor.Read();
        cout << "Rcv <= " << msgRcv->ToString() << endl << flush;

        if (msgRcv->CompareID(MESSAGE_MONITOR_LOST)) {
            delete(msgRcv);
            exit(-1);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_COM_OPEN)) {
            rt_sem_v(&sem_openComRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITHOUT_WD)) {
            rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
            modeRobot = MESSAGE_ROBOT_START_WITHOUT_WD;
            cout << "RELEASED" << endl << flush;
            rt_mutex_release(&mutex_modeWD);
            rt_sem_broadcast(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_START_WITH_WD)) {
            rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
            modeRobot = MESSAGE_ROBOT_START_WITH_WD;
            rt_mutex_release(&mutex_modeWD);
            rt_sem_broadcast(&sem_startRobot);
        } else if (msgRcv->CompareID(MESSAGE_ROBOT_GO_FORWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_BACKWARD) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_LEFT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_GO_RIGHT) ||
                msgRcv->CompareID(MESSAGE_ROBOT_STOP)) {

            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            move = msgRcv->GetID();
            rt_mutex_release(&mutex_move);
        }
        delete(msgRcv); // mus be deleted manually, no consumer
    }
}

/**
 * @brief Thread opening communication with the robot.
 */
void Tasks::OpenComRobot(void *arg) {
    int status;
    int err;

    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task openComRobot starts here                                                  */
    /**************************************************************************************/
    while (1) {
        rt_sem_p(&sem_openComRobot, TM_INFINITE);
        cout << "Open serial com (";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        status = robot.Open();
        rt_mutex_release(&mutex_robot);
        cout << status;
        cout << ")" << endl << flush;

        Message * msgSend;
        if (status < 0) {
            msgSend = new Message(MESSAGE_ANSWER_NACK);
        } else {
            msgSend = new Message(MESSAGE_ANSWER_ACK);
        }
        WriteInQueue(&q_messageToMon, msgSend); // msgSend will be deleted by sendToMon
    }
}

/**
 * @brief Thread starting the communication with the robot.
 */
void Tasks::StartRobotTask(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    /**************************************************************************************/
    /* The task startRobot starts here                                                    */
    /**************************************************************************************/
    while (1) {
        
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        rt_mutex_acquire(&mutex_comrobot_lost, TM_INFINITE);
        comRobot_Lost=0;
        rt_mutex_release(&mutex_comrobot_lost);

        Message * msgSend;
        rt_mutex_acquire(&mutex_modeWD, TM_INFINITE);
        int mode_robot = modeRobot;
        cout << "MODE_ROBOT VALUE :" << mode_robot << endl << flush; 
        rt_mutex_release(&mutex_modeWD);

        if(mode_robot == MESSAGE_ROBOT_START_WITHOUT_WD) {
            cout << "Start robot without watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;

            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);
            }
        }
        else if (mode_robot == MESSAGE_ROBOT_START_WITH_WD) {
            cout << "Start robot with watchdog (";
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            msgSend = robot.Write(robot.StartWithoutWD());
            rt_mutex_release(&mutex_robot);
            cout << msgSend->GetID();
            cout << ")" << endl;

            cout << "Movement answer: " << msgSend->ToString() << endl << flush;
            WriteInQueue(&q_messageToMon, msgSend);  // msgSend will be deleted by sendToMon

            if (msgSend->GetID() == MESSAGE_ANSWER_ACK) {
                rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
                robotStarted = 1;
                rt_mutex_release(&mutex_robotStarted);

                rt_mutex_acquire(&mutex_compteurWD, TM_INFINITE);
                compteurWD = 0;
                rt_mutex_release(&mutex_compteurWD);
                rt_sem_v(&sem_reloadWD);
            }
        } 
    }
}


void Tasks::ReloadWD(void *arg) {
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    Message * msgSend;

    rt_task_set_periodic(NULL, TM_NOW, 1000000000);
    while(1){
        rt_task_wait_period(NULL);
        rt_sem_p(&sem_reloadWD, TM_INFINITE);
        cout << "Periodic update WD";
        rt_mutex_acquire(&mutex_robot, TM_INFINITE);
        msgSend = robot.Write(new Message((MessageID)MESSAGE_ROBOT_RELOAD_WD));
        rt_mutex_release(&mutex_robot);
        if(msgSend->GetID() == MESSAGE_ANSWER_ACK){
            rt_mutex_acquire(&mutex_compteurWD, TM_INFINITE);
            compteurWD--;
            cout << "compterWD is " << compteurWD << endl << flush;
            rt_mutex_release(&mutex_compteurWD);
        }
        else if(msgSend->GetID() == MESSAGE_ANSWER_NACK) {
            rt_mutex_acquire(&mutex_compteurWD, TM_INFINITE);
            compteurWD++;
            cout << "compterWD is " << compteurWD << endl << flush;
            if (compteurWD == 3) {
                cout << "Arret car WD" << endl << flush;
                // Par précaution on arrête le robot
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Stop();
                rt_mutex_release(&mutex_robot);
                // Stopper com avec robot
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                robot.Reset();
                robot.Close();
                rt_mutex_release(&mutex_robot);
            }
            rt_mutex_release(&mutex_compteurWD);
        }
    }
}


/**
 * @brief Thread handling control of the robot.
 */
void Tasks::MoveTask(void *arg) {
    int rs;
    int cpMove;
    
    cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
    // Synchronization barrier (waiting that all tasks are starting)
    rt_sem_p(&sem_barrier, TM_INFINITE);
    
    /**************************************************************************************/
    /* The task starts here                                                               */
    /**************************************************************************************/
    rt_task_set_periodic(NULL, TM_NOW, 100000000);

    while (1) {
        rt_task_wait_period(NULL);
        cout << "Periodic movement update";
        rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
        rs = robotStarted;
        rt_mutex_release(&mutex_robotStarted);
        if (rs == 1) {
            rt_mutex_acquire(&mutex_move, TM_INFINITE);
            cpMove = move;
            rt_mutex_release(&mutex_move);
            
            cout << " move: " << cpMove;
            ;
             WriteInQueue(&q_messageToRobot,(new Message((MessageID)cpMove)));
        }
        cout << endl << flush;
    }
}

/**
 * Write a message in a given queue
 * @param queue Queue identifier
 * @param msg Message to be stored
 */
void Tasks::WriteInQueue(RT_QUEUE *queue, Message *msg) {
    int err;
    if ((err = rt_queue_write(queue, (const void *) &msg, sizeof ((const void *) &msg), Q_NORMAL)) < 0) {
        cerr << "Write in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in write in queue"};
    }
}

/**
 * Read a message from a given queue, block if empty
 * @param queue Queue identifier
 * @return Message read
 */
Message *Tasks::ReadInQueue(RT_QUEUE *queue) {
    int err;
    Message *msg;

    if ((err = rt_queue_read(queue, &msg, sizeof ((void*) &msg), TM_INFINITE)) < 0) {
        cout << "Read in queue failed: " << strerror(-err) << endl << flush;
        throw std::runtime_error{"Error in read in queue"};
    }/** else {
        cout << "@msg :" << msg << endl << flush;
    } /**/

    return msg;
}


const string MESSAGE_ID_STRING[] = {
    "Empty",
    "Log",
    "Answer [Acknowledge]",
    "Answer [Not Acknowledge]",
    "Answer [Command timeout]",
    "Answer [Command unknown]",
    "Answer [Command error]",
    "Answer [Communication error]",
    "Monitor connection lost",
    "Open serial com",
    "Close serial com",
    "Open camera",
    "Close camera",
    "Ask for arena",
    "Arena confirmed",
    "Arena infirmed",
    "Compute position",
    "Stop compute position",
    "Position",
    "Image",
    "Robot ping",
    "Robot reset",
    "Robot start with watchdog",
    "Robot start without watchdog",
    "Robot reload watchdog",
    "Robot move",
    "Robot turn",
    "Robot go forward",
    "Robot go backward",
    "Robot go left",
    "Robot go right",
    "Robot stop",
    "Robot poweroff",
    "Robot get battery",
    "Robot battery level",
    "Robot get state",
    "Robot current state",
    "Robot state [Not busy]",
    "Robot state [Busy]"
};

void Tasks::SendToRobotTask(void* arg) {
    while(1){
        Message *msg;

        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
        // Synchronization barrier (waiting that all tasks are starting)
        rt_sem_p(&sem_barrier, TM_INFINITE);

        /**************************************************************************************/
        /* The task sendToRobot starts here                                                     */
        /**************************************************************************************/
        rt_sem_p(&sem_startRobot, TM_INFINITE);
        Message * message_response_robot;
        MessageState checked_sent_message;
        while (1) {
                cout << "wait msg to send" << endl << flush;
                msg = ReadInQueue(&q_messageToRobot);
                cout << "Send msg to robot: " << msg->ToString() << endl << flush;
                rt_mutex_acquire(&mutex_robot, TM_INFINITE);
                message_response_robot=robot.Write(msg); // The message is deleted with the Write
                rt_mutex_release(&mutex_robot);
                checked_sent_message=Check_ComRobot(message_response_robot);
                cout << "Send msg to robot OK "<< endl;
                if(checked_sent_message == MESSAGE_SENT_TO_ROBOT){
                    WriteInQueue(&q_messageToMon,message_response_robot);
                    cout << "Send msg to mon OK "<< endl;
                }else if(comRobot_FailCounter==3){
                    break;
                }
            }
        }
    }

MessageState Tasks::Check_ComRobot(Message* message){
    MessageState find = MESSAGE_SENT_TO_ROBOT;
    cout << "11111111111111111111111111111" << endl<<flush;
    cout << MESSAGE_ID_STRING[message->GetID()] << endl;
    if (message->CompareID(MESSAGE_ANSWER_ROBOT_TIMEOUT)){
        cout << "HELLO LES AMIS" << endl<<flush;
        rt_mutex_acquire(&mutex_comrobot_failcounter, TM_INFINITE);
        ++comRobot_FailCounter;
        rt_mutex_release(&mutex_comrobot_failcounter);
        find=MESSAGE_NOT_SENT_TO_ROBOT;
        rt_mutex_acquire(&mutex_comrobot_failcounter, TM_INFINITE);
        if (comRobot_FailCounter==3){
            rt_mutex_release(&mutex_comrobot_failcounter);
            // Closing communication with the robot
            rt_mutex_acquire(&mutex_robot, TM_INFINITE);
            cout << "DAZJIOOOOOOOOOOOOOOOOOOOZ"<< endl<<flush;
            robot.Reset();
            rt_mutex_release(&mutex_robot);    
            // Setting failcounter global variable to 0
            rt_mutex_acquire(&mutex_comrobot_failcounter, TM_INFINITE);
            cout << "DAZJIOOOOOZ"<< endl <<flush;
            comRobot_FailCounter=0;
            rt_mutex_release(&mutex_comrobot_failcounter);  
            
            // Setting robotStarted global variable to 0.
            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            cout << "DAZJIOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOZ"<< endl << flush;
            robotStarted = 0;
            rt_mutex_release(&mutex_robotStarted);
            find = CONNECTION_LOST_WITH_ROBOT;
            rt_mutex_acquire(&mutex_comrobot_lost, TM_INFINITE);
            cout << "VVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVVV"<< endl << flush;
            comRobot_Lost = 1;
            rt_mutex_release(&mutex_comrobot_lost);
            Message * errorMessage = new Message(MESSAGE_ANSWER_COM_ERROR);
            WriteInQueue(&q_messageToMon,errorMessage);
            rt_queue_flush(&q_messageToRobot);

        }
        
    }else{
        rt_mutex_acquire(&mutex_comrobot_failcounter, TM_INFINITE);
        comRobot_FailCounter=0;
        rt_mutex_release(&mutex_comrobot_failcounter);
    }
    return find;
}
/**
 * @brief Thread handling control of the robot.
 */
void Tasks::BatteryTask(void *arg) {
    
    while(1){
        int rs;

        cout << "Start " << __PRETTY_FUNCTION__ << endl << flush;
        // Synchronization barrier (waiting that all tasks are starting)
        rt_sem_p(&sem_barrier, TM_INFINITE);
        cout << "battery_synchronized" << endl;

        rt_sem_p(&sem_startRobot, TM_INFINITE);
            cout << "battery started" << endl;

        /**************************************************************************************/
        /* The task starts here                                                               */
        /**************************************************************************************/
        rt_task_set_periodic(NULL, TM_NOW, 500000000);


        Message * bonjour_batterie;
        

        while (1) {
            rt_task_wait_period(NULL);
            // Instruction moved inside while to be able restarting program without lauching task again
            bool started = false;

            rt_mutex_acquire(&mutex_robotStarted, TM_INFINITE);
            started = (bool) robotStarted;
            rt_mutex_release(&mutex_robotStarted);

            if(started){
                
                bonjour_batterie = new Message ((MessageID) MESSAGE_ROBOT_BATTERY_GET);
                cout << "battery add in queue" << endl;
                WriteInQueue(&q_messageToRobot,bonjour_batterie);
            }
            
            rt_mutex_acquire(&mutex_comrobot_lost, TM_INFINITE);
            if (comRobot_Lost == 1){
                 rt_mutex_release(&mutex_comrobot_lost);
                 cout << "arret robot" << endl;
                 break;
            } else {
                rt_mutex_release(&mutex_comrobot_lost);
            }
        }
    }
}