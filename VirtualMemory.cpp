#include "VirtualMemory.h"
#include "PhysicalMemory.h"
#include <utility>
#include <cmath>

// tuple of (frame reference, frame index)
typedef std::pair<uint64_t, uint64_t> FrameTuple;

/**
 * get page index, return frame index of that page. is path is missing to that page, path is
 * being built
 * @param page index
 * @return frame index
 */
FrameTuple findFrameFromPage(uint64_t page);

void clearTable(uint64_t frameIndex)
{
    for (uint64_t i = 0; i < PAGE_SIZE; ++ i)
    {
        PMwrite(frameIndex * PAGE_SIZE + i, 0);
    }
}

/**
 * calc cyclic distance
 * @param pageA
 * @param pageB
 * @return dist
 */
double cyclicDistanceCalc(uint64_t &pageA, uint64_t pageB)
{
    auto a = (int64_t) pageA;
    auto b = (int64_t) pageB;
    return std::fmin((int64_t) NUM_PAGES - std::abs(a - b), std::abs(a - b));
}

/**
 * @brief DFS search - finds empty frame. if yes - delete the reference to it. if not - update page
 * to evict
 * and maxFrameIndex
 * @param frameIndex sub tree root
 * @param currPageVirtualAddress
 * @param maxFrameIndex maximal frame index discovered so far
 * @param cyclicDistance maximal cyclic distance discovered so far
 * @param pageToEvict curr best page to evict
 * @param targetPage page to be entered eventually
 * @param treeDepth what depth is the root in the tree
 * @param lastAddedFrame if in the middle of path building, last added frame
 * @param father sub tree father
 * @return 0 if no available frame, frame index otherwise
 */
uint64_t
traversTree(uint64_t frameIndex, uint64_t currPageVirtualAddress, uint64_t &maxFrameIndex,
            double &cyclicDistance,
            uint64_t &pageToEvict, uint64_t &targetPage, int treeDepth, uint64_t lastAddedFrame,
            uint64_t father)
{
    // check the page frame num
    if (frameIndex > maxFrameIndex)
    {
        maxFrameIndex = frameIndex;
    }
    // reached leaf - real page
    if (treeDepth == TABLES_DEPTH)
    {
        double dist = cyclicDistanceCalc(targetPage, (int64_t) currPageVirtualAddress);
        if (dist > cyclicDistance)
        {
            cyclicDistance = dist;
            pageToEvict = currPageVirtualAddress;
        }
        return 0;
    }
    bool emptyFrame = true;
    word_t nextFrameIndex;
    for (int i = 0; i < PAGE_SIZE; ++ i)
    {
        PMread(((uint64_t) frameIndex) * PAGE_SIZE + i, &nextFrameIndex);
        if (nextFrameIndex)
        {
            emptyFrame = false;
            uint64_t returnFrame = traversTree((uint64_t) nextFrameIndex,
                                               (currPageVirtualAddress << OFFSET_WIDTH) + i,
                                               maxFrameIndex,
                                               cyclicDistance, pageToEvict, targetPage,
                                               treeDepth + 1, lastAddedFrame,
                                               ((uint64_t) frameIndex) * PAGE_SIZE + i);
            // check if found an empty frame
            if (returnFrame)
            {
                return returnFrame;
            }
        }
    }
    if (emptyFrame && (frameIndex != lastAddedFrame))
    {
        PMwrite(father, 0);
        return frameIndex;

    }
    return 0;
}

/**
 * @brief find frame to fill and prepare accordingly
 * @param lastAddedFrame if in the middle of path building, last added frame
 * @return frame index
 */
uint64_t getFrameToFill(uint64_t lastAddedFrame)
{
    uint64_t pageToEvict;
    uint64_t targetPage; // todo need to be reset
    double bestCyclicDistance = 0;
    uint64_t max_frame = 0;
    uint64_t emptyPage = traversTree(0, 0, max_frame, bestCyclicDistance, pageToEvict, targetPage,
                                     0, lastAddedFrame, 0);
    // 1 prio - if found empty with 0
    if (emptyPage)
    {
        // father cleared inside traverseTree
        return emptyPage;
    }
        // 2 prio - if there is unused frame
    else if (max_frame + 1 < NUM_FRAMES)
    {
        clearTable((uint64_t) max_frame + 1);
        return max_frame + 1;
    }
    // 3 prio - swap page
    FrameTuple frameTuple = findFrameFromPage(pageToEvict);
    PMevict(frameTuple.second, pageToEvict);
    clearTable(frameTuple.second);
    // clear father
    PMwrite(frameTuple.first, 0);
    return frameTuple.second;
}


FrameTuple findFrameFromPage(uint64_t page)
{
    word_t nextFrameIndex = 0;
    // uint64_t previousFrame; // todo check
    uint64_t father = 0;
    uint64_t lastAddedFrame = 0;
    bool shouldRestore = false;
    for (int i = 0; i < TABLES_DEPTH; i ++)
    {
        auto currOffsetIndex = (uint64_t) (page >> (OFFSET_WIDTH * (TABLES_DEPTH - i - 1)) &
                                           (PAGE_SIZE - 1));

        father = PAGE_SIZE * nextFrameIndex + currOffsetIndex;
        PMread(father, &nextFrameIndex);
        if (! nextFrameIndex)
        {
            shouldRestore = true;
            lastAddedFrame = getFrameToFill(lastAddedFrame);
            PMwrite(father, (word_t) lastAddedFrame);
            nextFrameIndex = (word_t) lastAddedFrame;
        }
    }
    if (shouldRestore)
    {
        PMrestore((uint64_t) nextFrameIndex, page);
    }
    return {father, (uint64_t) nextFrameIndex};


}


void VMinitialize()
{
    clearTable(0);
}

int VMread(uint64_t virtualAddress, word_t *value)
{
    if (virtualAddress >= VIRTUAL_MEMORY_SIZE)
    {
        return 0;
    }
    FrameTuple frameTuple = findFrameFromPage(virtualAddress >> OFFSET_WIDTH);
    uint64_t frame = frameTuple.second;
    auto offset = (uint64_t) (virtualAddress & (PAGE_SIZE - 1));
    PMread(PAGE_SIZE * frame + offset, value);
    return 1;
}


int VMwrite(uint64_t virtualAddress, word_t value)
{
    if (virtualAddress >= VIRTUAL_MEMORY_SIZE)
    {
        return 0;
    }
    FrameTuple frameTuple = findFrameFromPage(virtualAddress >> OFFSET_WIDTH);
    uint64_t frame = frameTuple.second;
    auto offset = (uint64_t) (virtualAddress & (PAGE_SIZE - 1));
    PMwrite(PAGE_SIZE * frame + offset, value);
    return 1;
}
