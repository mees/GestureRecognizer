/*
* Copyright (c) 2012, Richard Hertel, Oier Mees
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above copyright
*      notice, this list of conditions and the following disclaimer in the
*      documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "RPSGame.h"

using namespace std;
// kdTreeManager has no standard constructor, so it should be initialized with its path as argument.
RPSGame::RPSGame() : kdTreeManager("Training/") {
	srand(time(NULL));
}

// kdTreeManager has no standard constructor, so we use the standard copy-constructor to initialize it.
RPSGame::RPSGame(KdTreeManager & kdTreeManager) : kdTreeManager(kdTreeManager) {
	srand(time(NULL));
}

RPSGame::~RPSGame() {
	// TODO Auto-generated destructor stub
}

KdTreeManager::GestureType RPSGame::generateRandomGesture()
{
	KdTreeManager::GestureType resultGesture = KdTreeManager::INVALID;
	int randGestureNumber = rand() % 3;
	switch(randGestureNumber)
	{
	case 0: return KdTreeManager::SCISSOR; break;
	case 1: return KdTreeManager::ROCK; break;
	case 2: return KdTreeManager::PAPER; break;
	default: return KdTreeManager::INVALID; break;
	}
	return resultGesture;
}

RPSGame::Winner RPSGame::calculateWinner(KdTreeManager::GestureType cpuGesture, KdTreeManager::GestureType humanGesture)
{
	if(cpuGesture == humanGesture) return RPSGame::TIE;
	if(cpuGesture == KdTreeManager::SCISSOR && humanGesture == KdTreeManager::PAPER ) return RPSGame::CPU;
	if(cpuGesture == KdTreeManager::PAPER && humanGesture == KdTreeManager::ROCK ) return RPSGame::CPU;
	if(cpuGesture == KdTreeManager::ROCK && humanGesture == KdTreeManager::SCISSOR ) return RPSGame::CPU;
	if(cpuGesture == KdTreeManager::SCISSOR && humanGesture == KdTreeManager::ROCK ) return RPSGame::HUMAN;
	if(cpuGesture == KdTreeManager::ROCK && humanGesture == KdTreeManager::PAPER ) return RPSGame::HUMAN;
	if(cpuGesture == KdTreeManager::PAPER && humanGesture == KdTreeManager::SCISSOR ) return RPSGame::HUMAN;

	return RPSGame::INVALID;
}

std::string RPSGame::winnerToString(RPSGame::Winner winner)
{
	if(winner == RPSGame::CPU) return "Cpu wins!";
	if(winner == RPSGame::HUMAN) return "You win!";
	if(winner == RPSGame::TIE) return "Tie!";
	return "Winner invalid";
}

void RPSGame::processPointCloud(pcl::PointCloud<pcl::PointXYZ> pointCloud)
{
	KdTreeManager::GestureType humanGesture = kdTreeManager.getNearestGestureType(pointCloud);

	KdTreeManager::GestureType cpuGesture = generateRandomGesture();

	cout << "My Gesture: " << kdTreeManager.gestureTypeToString(cpuGesture) << endl;
	cout << "Your Gesture: " << kdTreeManager.gestureTypeToString(humanGesture) << endl;

	RPSGame::Winner winner = calculateWinner(cpuGesture,humanGesture);
	cout << winnerToString(winner) << endl;

}
