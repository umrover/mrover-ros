<template>
    <div>
        {{ text }}
    </div>
</template>
<script>
    export default {
        name: "NoteComp",
        props: {
            text: {
                type: String,
                required: true,
            },
        },
    }
</script>
<style scoped>
    div {
        display: flex;
        width: 200px;
        min-height: 100px;
        padding: 10px;
        background-color: beige;
        border: none;
        box-shadow: 0 0 5px rgba(0, 0, 0, 0.4);
        border-radius: 5px;
        font-family: Avenir, Helvetica, Arial, sans-serif;
        font-size: 1em;
        margin: 5px;
    }
</style>