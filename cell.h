#ifndef CELL_H
#define CELL_H

typedef struct cell {
        bool valid;
        void (*update)(void *value, void **args);
        size_t num_args;
        struct cell_t **args;
        struct cell_list *dependents;
} cell_t;

struct cell_list {
        struct cell *cell;
        struct cell_list *next;
};

cell_t
create_cell(size_t value_size, void (*update)(void *value, void **args), ...)
{
        cell_t *cell;
        va_list ap1, ap2;
        size_t num_args = 0;

        va_start(ap1, update);
        va_copy(ap2, ap1);

        while (va_arg(ap, cell_t*) != NULL)
                num_args++;
        
        va_end(ap1);

        cell = malloc(sizeof(cell_t) + value_size + num_args * sizeof(cell_t*));

        cell->valid = false;
        cell->update = update;
        cell->num_args = num_args;
        cell->args = (cell_t**)(((char*)(cell + 1)) + value_size);
        cell->dependents = NULL;

        return cell;
}

#endif /* CELL_H */
