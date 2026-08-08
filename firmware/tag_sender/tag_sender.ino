//최종 송신부
//EEPROM을 이용해서 reset버튼 눌러도 마지막 좌표를 잊지 않고 시작좌표로 설정가능 
#include <EEPROM.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"

// RGB 센서 초기화
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define MAP_WIDTH 10
#define MAP_HEIGHT 10
#define MAX_NODES 100 // 노드 풀 크기

typedef struct {
    int8_t x, y; // int 대신 int8_t 사용하여 메모리 사용 줄이기
} Point;

typedef struct Node {
    Point point;
    uint16_t g, h; // int 대신 uint16_t 사용하여 메모리 사용 줄이기
    struct Node* parent;
} Node;

Node nodePool[MAX_NODES];
int nodePoolIndex = 0;

Node* openList[MAX_NODES];
int openListSize = 0;
Node* closedList[MAX_NODES];
int closedListSize = 0;

const char grid[MAP_HEIGHT][MAP_WIDTH] PROGMEM = {
    {'.', '.', '.', '.', '.', '#', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '#', '.', '.', '.', '.'},
    {'.', '.', '.', '#', '#', '#', '.', '.', '.', '.'},
    {'.', '.', '.', '#', '#', '#', '.', '.', '.', '.'},
    {'.', '.', '.', '#', '#', '#', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.', '.', '.', '.', '.'},
    {'.', '.', '.', '.', '.', '.', '.', '.', '.', '.'}
};

char pathGrid[MAP_HEIGHT][MAP_WIDTH];
Point currentEnd = {1, 1}; // 전역 변수로 선언

void initializePathGrid() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            pathGrid[y][x] = pgm_read_byte(&(grid[y][x]));
        }
    }
}

void printMap() {
    for (int y = 0; y < MAP_HEIGHT; y++) {
        for (int x = 0; x < MAP_WIDTH; x++) {
            Serial.print(pathGrid[y][x]);
            Serial.print(' ');
        }
        Serial.println();
    }
}

int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y); // 맨해튼 거리
}

Node* createNode(Point point, Node* parent, uint16_t g, uint16_t h) {
    if (nodePoolIndex >= MAX_NODES) {
        return NULL; // 노드 풀 오버플로 방지
    }
    Node* node = &nodePool[nodePoolIndex++];
    node->point = point;
    node->g = g;
    node->h = h;
    node->parent = parent;
    return node;
}

bool isInList(Node* list[], int size, Point point) {
    for (int i = 0; i < size; i++) {
        if (list[i]->point.x == point.x && list[i]->point.y == point.y) {
            return true;
        }
    }
    return false;
}

void swapNodes(Node** a, Node** b) {
    Node* temp = *a;
    *a = *b;
    *b = temp;
}

void push(Node* node) {
    if (openListSize >= MAX_NODES) return;
    openList[openListSize] = node;
    int i = openListSize;
    openListSize++;
    while (i != 0 && (openList[i]->g + openList[i]->h) < (openList[(i - 1) / 2]->g + openList[(i - 1) / 2]->h)) {
        swapNodes(&openList[i], &openList[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

Node* pop() {
    if (openListSize <= 0) return NULL;
    if (openListSize == 1) {
        openListSize--;
        return openList[0];
    }

    Node* root = openList[0];
    openList[0] = openList[openListSize - 1];
    openListSize--;

    int i = 0;
    while (i * 2 + 1 < openListSize) {
        int smallest = i;
        if (openList[i * 2 + 1]->g + openList[i * 2 + 1]->h < openList[smallest]->g + openList[smallest]->h) {
            smallest = i * 2 + 1;
        }
        if (i * 2 + 2 < openListSize && openList[i * 2 + 2]->g + openList[i * 2 + 2]->h < openList[smallest]->g + openList[smallest]->h) {
            smallest = i * 2 + 2;
        }
        if (smallest == i) break;
        swapNodes(&openList[i], &openList[smallest]);
        i = smallest;
    }

    return root;
}

bool isCollision(Point point) {
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int x = point.x + dx;
            int y = point.y + dy;
            if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
                if (pgm_read_byte(&(grid[y][x])) == '#') {
                    return true;
                }
            }
        }
    }
    return false;
}

void markPath(Node* node) {
    while (node != NULL) {
        pathGrid[node->point.y][node->point.x] = 'o'; // 표식을 pathGrid에 저장
        node = node->parent;
    }
}

void sendPath(Node* node) {
    Node* pathStack[MAX_NODES];
    int pathStackSize = 0;
    while (node != NULL) {
        pathStack[pathStackSize++] = node;
        node = node->parent;
    }

    Serial.print("Path: ");
    for (int i = pathStackSize - 1; i >= 0; i--) {
        Serial.print(pathStack[i]->point.x);
        Serial.print(",");
        Serial.print(pathStack[i]->point.y);
        if (i != 0) {
            Serial.print(";");
        }
        delay(10);
    }
    Serial.println(); // 전체 경로를 한 번에 전송하고 줄 바꿈
}

void findPath(Point start, Point end) {
    nodePoolIndex = 0;
    openListSize = 0;
    closedListSize = 0;

    Node* startNode = createNode(start, NULL, 0, heuristic(start, end));
    push(startNode);
    Node* currentNode = NULL;

    while (openListSize > 0) {
        currentNode = pop();

        if (currentNode->point.x == end.x && currentNode->point.y == end.y) {
            markPath(currentNode);
            Serial.println("Path found!");
            sendPath(currentNode);
            currentEnd = end; // 현재 끝 지점을 업데이트
            return;
        }

        closedList[closedListSize++] = currentNode;

        Point directions[] = {
            {0, -1}, {1, 0}, {0, 1}, {-1, 0}
        };
        for (int i = 0; i < 4; i++) {
            Point newPoint = { currentNode->point.x + directions[i].x, currentNode->point.y + directions[i].y };

            if (newPoint.x < 0 || newPoint.x >= MAP_WIDTH || newPoint.y < 0 || newPoint.y >= MAP_HEIGHT) {
                continue;
            }

            if (pgm_read_byte(&(grid[newPoint.y][newPoint.x])) == '#' || isCollision(newPoint)) {
                continue;
            }

            if (isInList(closedList, closedListSize, newPoint) || isInList(openList, openListSize, newPoint)) {
                continue;
            }

            int newG = currentNode->g + 10;
            Node* successor = createNode(newPoint, currentNode, newG, heuristic(newPoint, end));

            if (successor != NULL) {
                push(successor);
            }
        }
    }

    Serial.println("No path found");
}

void saveCurrentEndToEEPROM() {
    EEPROM.put(0, currentEnd);
}

void loadCurrentEndFromEEPROM() {
    EEPROM.get(0, currentEnd);
}

void setup() {
    Serial.begin(115200);
    Wire.begin();

    if (tcs.begin()) {
        Serial.println("TCS34725 found and initialized");
    } else {
        Serial.println("No TCS34725 found ... check your connections");
        while (1);
    }
    
    // EEPROM에서 마지막 끝 지점 읽기
    loadCurrentEndFromEEPROM();
    
    float red, green, blue;
    tcs.getRGB(&red, &green, &blue);

    Serial.print("R:\t");
    Serial.print(int(red));
    Serial.print(" G:\t");
    Serial.print(int(green));
    Serial.print(" B:\t");
    Serial.println(int(blue));

    initializePathGrid();
    Point start = currentEnd; // 이전 끝 지점을 새로운 시작 지점으로 설정

    if (red > 100) {
        currentEnd = {4, 7};
        findPath(start, currentEnd);
    } else if (green > 110) {
        currentEnd = {7, 2};
        findPath(start, currentEnd);
    } else if (blue > 100) {
        currentEnd = {8, 7};
        findPath(start, currentEnd);
    }
    else
    {
      currentEnd = {1, 1};
      findPath(start, currentEnd);
    }

    // 새로운 끝 지점을 EEPROM에 저장
    saveCurrentEndToEEPROM();
}

void loop() {
    // do nothing
}