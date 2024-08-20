using System.Collections.Generic;

public class PriorityQueue<T>
{
    private List<(T item, float priority)> elements = new List<(T, float)>();

    public int Count => elements.Count;

    public void Enqueue(T item, float priority)
    {
        elements.Add((item, priority));
        elements.Sort((a, b) => a.priority.CompareTo(b.priority));
    }

    public T Dequeue()
    {
        if (elements.Count == 0)
        {
            throw new System.InvalidOperationException("Queue is empty");
        }

        T item = elements[0].item;
        elements.RemoveAt(0);
        return item;
    }
}