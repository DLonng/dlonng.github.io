---
title: 登龙的算法课：LeetCode 206. 反转链表
date: 2020-04-29 16:00:00
---
# 登龙的算法课：LeetCode 206. 反转链表

***
> 版权声明：本文为 {{ site.name }} 原创文章，可以随意转载，但必须在明确位置注明出处！



面试高频算法：反转一个链表。今天给出 3 种方法，助你搞定面试官！

## 一、题目描述（@LeetCode）

反转一个单链表，示例:

- 输入: 1->2->3->4->5->NULL
- 输出: 5->4->3->2->1->NULL

## 二、解题思路

### 2.1 迭代法

步骤如下：
1. 设置双指针 pre，cur
2. 设置 cur_next 指针保存 cur 的下一个节点，防止断链
3. 断开 cur 节点的链接
4. 将 cur 的 next 指向前面的 pre 节点
5. pre 指针后移
6. cur 指针后移

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/reverse_list1.png)


```cpp
class Solution {
  public:
    ListNode* reverseList(ListNode* head) {
      // 1. 定义双指针
      ListNode *pre = nullptr;
      ListNode *cur = head;

      // 2. 定义用于保存 cur->next 的指针
      ListNode *cur_next = nullptr;

      // 3. 遍历直到尾部空节点结束
      while (cur != nullptr) {
        // 4. 必须先保存 next，不然链表会断开
        cur_next = cur->next;

        // 5. 逆转当前节点
        cur->next = pre;

        // 6. 双指针后移，注意要先移动 pre 节点哦
        pre = cur;
        cur = cur_next;
      }
    
      // 7. 因为循环结束时，cur == nullptr，pre 指向最后一个节点，也就是逆转后的头结点
      return pre;
    }
};
```

#### 复杂度分析

- 时间复杂度：O(n)，遍历 n 个节点
- 空间复杂度：O(1)，不使用额外空间

### 2.2 前递归

1. 设置 pre，cur 指针
2. 递归结束条件是 cur 为空，返回 pre 节点，比如遍历到尾节点
3. 每一次递归将当前节点 cur 的 next 链接到 pre 节点上
4. 递归处理下一个节点，注意参数有变化


![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/reverse_list2.png)

```cpp
class Solution {
  public:
    ListNode* reverseList(ListNode* head) {
      // 因为递归需要使用 pre 指针，而参数只有一个 head，所以用另一个带 pre 和 head 函数来执行递归
      return reverse_list(nullptr, head);
    }

    ListNode* reverse_list(ListNode* pre, ListNode* cur) {
      // 1. 当前节点为 nullptr 则返回前一节点 pre
      if (cur == nullptr) {
        return pre;
      } else {
        // 2. 保存 cur->next 节点防止断链
        ListNode* cur_next = cur->next;

        // 3. 逆转当前节点
        cur->next = pre;

        // 4. 递归逆转下一节点
        return reverse_list(cur, cur_next);
      }
    }
};
```

#### 复杂度分析

- 时间复杂度：O(n)，遍历 n 个节点
- 空间复杂度：O(n)，最多递归调用 n 次，递归调用栈深度最大为 n 层

### 2.3 尾递归

尾递归分为 2 步：

1. 先递归到达最后一个节点，每次处理的当前节点为 head
2. 将后面节点 head->next 的 next 链接到前面节点 head
2. 断开当前节点的 next 链接：head—>next = nullptr
3. 递归返回处理一个节点

![](https://dlonng.oss-cn-shenzhen.aliyuncs.com/blog/reverse_list3.png)


```cpp
class Solution {
public:
  ListNode* reverseList(ListNode* head) {
    // 1. head 为空或者递归到最后一个节点返回
    if (head == nullptr || head->next == nullptr)
      return head;
    
    // 2. 当触发递归结束条件后，cur 节点为倒数第二个节点
    ListNode* cur = reverseList(head->next);

    // 3. 逆转逻辑，看图理解
    head->next->next = head;
    head->next = nullptr;

    return cur;
  }
};
```

#### 复杂度分析

- 时间复杂度：O(n)，遍历 n 个节点
- 空间复杂度：O(n)，最多递归调用 n 次，递归调用栈深度最大为 n 层

LeetCode 链接：



[https://leetcode-cn.com/problems/reverse-linked-list/solution/deng-long-de-suan-fa-ke-leetcode-206-fan-zhuan-lia/](https://leetcode-cn.com/problems/reverse-linked-list/solution/deng-long-de-suan-fa-ke-leetcode-206-fan-zhuan-lia/)


> {{ site.prompt }}

<div  align="center">
<img src="https://dlonng.com/images/wechart.jpg" width = "200" height = "200"/>