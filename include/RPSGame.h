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

#ifndef RPSGAME_H_
#define RPSGAME_H_
#include "KdTreeManager.h"

class RPSGame {
	enum Winner {
		TIE = 0,
		HUMAN = 1,
		CPU = 2,

		INVALID = 42
	};

public:
	RPSGame();
	RPSGame(KdTreeManager & kdTreeManager);
	virtual ~RPSGame();
	KdTreeManager::GestureType generateRandomGesture();
	RPSGame::Winner calculateWinner(KdTreeManager::GestureType cpuGesture, KdTreeManager::GestureType humanGesture);
	void processPointCloud(pcl::PointCloud<pcl::PointXYZ> pointCloud);

private:
	KdTreeManager kdTreeManager;
	std::string winnerToString(RPSGame::Winner winner);
};

#endif /* RPSGAME_H_ */
